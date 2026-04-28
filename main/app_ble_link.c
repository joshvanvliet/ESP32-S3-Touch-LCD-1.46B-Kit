#include "app_ble_link.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_att.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_hci.h"
#include "host/ble_sm.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

/* Not declared by ble_store_config.h in ESP-IDF 5.3.x */
extern void ble_store_config_init(void);

#define APP_BLE_DEVICE_NAME "ESP Assistant"
#define APP_BLE_PACKET_MAX 244
#define APP_BLE_DEFAULT_NOTIFY_PAYLOAD_MAX 185

#ifndef BLE_ATT_ERR_INSUFFICIENT_AUTHEN
#define BLE_ATT_ERR_INSUFFICIENT_AUTHEN 0x05
#endif
#ifndef BLE_ATT_ERR_WRITE_NOT_PERMITTED
#define BLE_ATT_ERR_WRITE_NOT_PERMITTED 0x03
#endif
#ifndef BLE_ATT_ERR_CCCD_IMPROPERLY_CONFIGURED
#define BLE_ATT_ERR_CCCD_IMPROPERLY_CONFIGURED 0xFD
#endif
#ifndef BLE_ERR_REM_USER_CONN_TERM
#define BLE_ERR_REM_USER_CONN_TERM 0x13
#endif

static const char *TAG = "APP_BLE";
static const uint8_t APP_BLE_NOTIFY_RETRY_ATTEMPTS = 3;

static app_ble_callbacks_t s_callbacks;
static bool s_initialized;
static bool s_transport_connected;
static bool s_security_ready;
static bool s_app_ready;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t s_addr_type;
static uint16_t s_event_val_handle;
static uint16_t s_audio_val_handle;
static uint16_t s_audio_payload_max = APP_BLE_DEFAULT_NOTIFY_PAYLOAD_MAX;
static bool s_event_notify_enabled;
static bool s_audio_notify_enabled;
static uint32_t s_active_passkey;
static bool s_pairing_in_progress;
static app_device_state_t s_last_notified_state = APP_STATE_UNPAIRED;
static app_ble_audio_tx_stats_t s_audio_tx_stats;
static portMUX_TYPE s_audio_tx_lock = portMUX_INITIALIZER_UNLOCKED;

typedef enum {
    APP_CHAR_CONTROL = 0,
    APP_CHAR_EVENT = 1,
    APP_CHAR_AUDIO = 2,
    APP_CHAR_RESULT = 3,
    APP_CHAR_AUDIO_DOWNLINK = 4,
} app_ble_char_t;

static const ble_uuid128_t s_service_uuid = BLE_UUID128_INIT(
    0x00, 0x00, 0xD9, 0xB6, 0xF4, 0x30, 0xD4, 0x9E,
    0x51, 0x4F, 0x7F, 0x5D, 0x01, 0x00, 0x5A, 0x7F);

static const ble_uuid128_t s_control_uuid = BLE_UUID128_INIT(
    0x00, 0x00, 0xD9, 0xB6, 0xF4, 0x30, 0xD4, 0x9E,
    0x51, 0x4F, 0x7F, 0x5D, 0x02, 0x00, 0x5A, 0x7F);

static const ble_uuid128_t s_event_uuid = BLE_UUID128_INIT(
    0x00, 0x00, 0xD9, 0xB6, 0xF4, 0x30, 0xD4, 0x9E,
    0x51, 0x4F, 0x7F, 0x5D, 0x03, 0x00, 0x5A, 0x7F);

static const ble_uuid128_t s_audio_uuid = BLE_UUID128_INIT(
    0x00, 0x00, 0xD9, 0xB6, 0xF4, 0x30, 0xD4, 0x9E,
    0x51, 0x4F, 0x7F, 0x5D, 0x04, 0x00, 0x5A, 0x7F);

static const ble_uuid128_t s_result_uuid = BLE_UUID128_INIT(
    0x00, 0x00, 0xD9, 0xB6, 0xF4, 0x30, 0xD4, 0x9E,
    0x51, 0x4F, 0x7F, 0x5D, 0x05, 0x00, 0x5A, 0x7F);

static const ble_uuid128_t s_audio_downlink_uuid = BLE_UUID128_INIT(
    0x00, 0x00, 0xD9, 0xB6, 0xF4, 0x30, 0xD4, 0x9E,
    0x51, 0x4F, 0x7F, 0x5D, 0x06, 0x00, 0x5A, 0x7F);

static uint16_t app_le16_read(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void app_le16_write(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static int app_ble_notify_raw(uint16_t attr_handle, const uint8_t *data, uint16_t len)
{
    if (!s_transport_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return BLE_HS_ENOTCONN;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        return BLE_HS_ENOMEM;
    }

    return ble_gatts_notify_custom(s_conn_handle, attr_handle, om);
}

static int app_ble_notify_with_retry(uint16_t attr_handle, const uint8_t *data, uint16_t len, uint8_t *retries_out)
{
    uint8_t retries = 0;
    int rc = 0;

    for (uint8_t attempt = 0; attempt < APP_BLE_NOTIFY_RETRY_ATTEMPTS; ++attempt) {
        rc = app_ble_notify_raw(attr_handle, data, len);
        if (rc == 0) {
            if (retries_out) {
                *retries_out = retries;
            }
            return 0;
        }
        if (attempt + 1 < APP_BLE_NOTIFY_RETRY_ATTEMPTS) {
            retries++;
            vTaskDelay(1);
        }
    }

    if (retries_out) {
        *retries_out = retries;
    }
    return rc;
}

static int app_ble_send_fragmented(uint16_t attr_handle, const uint8_t *packet, uint16_t packet_len, uint8_t *retries_out)
{
    uint16_t max_payload = s_audio_payload_max;
    if (max_payload > APP_BLE_PACKET_MAX) {
        max_payload = APP_BLE_PACKET_MAX;
    }
    if (max_payload < 8) {
        max_payload = 8;
    }

    if (packet_len + 2 <= max_payload) {
        uint8_t buf[APP_BLE_PACKET_MAX];
        buf[0] = 0;
        buf[1] = 1;
        memcpy(&buf[2], packet, packet_len);
        return app_ble_notify_with_retry(attr_handle, buf, packet_len + 2, retries_out);
    }

    uint16_t chunk_data = max_payload - 2;
    uint8_t total_frags = (uint8_t)((packet_len + chunk_data - 1) / chunk_data);
    uint16_t offset = 0;
    uint8_t total_retries = 0;

    for (uint8_t frag = 0; frag < total_frags; ++frag) {
        uint16_t remain = packet_len - offset;
        uint16_t this_len = remain > chunk_data ? chunk_data : remain;
        uint8_t buf[APP_BLE_PACKET_MAX];
        buf[0] = frag;
        buf[1] = total_frags;
        memcpy(&buf[2], &packet[offset], this_len);
        uint8_t frag_retries = 0;
        int rc = app_ble_notify_with_retry(attr_handle, buf, this_len + 2, &frag_retries);
        if (rc != 0) {
            if (retries_out) {
                *retries_out = (uint8_t)(total_retries + frag_retries);
            }
            return rc;
        }
        total_retries = (uint8_t)(total_retries + frag_retries);
        offset += this_len;
    }

    if (retries_out) {
        *retries_out = total_retries;
    }
    return 0;
}

static void app_ble_advertise(void);
static uint32_t app_ble_generate_passkey(void);
static void app_ble_notify_passkey(uint32_t passkey);
static void app_ble_reset_link_state(void);
static void app_ble_mark_link_secured(void);
static void app_ble_mark_app_ready(void);
static void app_ble_handle_pairing_failed(void);
static void app_ble_terminate_connection(void);
static void app_ble_log_conn_params(uint16_t conn_handle, const char *where);
static void app_ble_request_low_latency_conn_params(uint16_t conn_handle, const char *where);

static void app_ble_log_conn_params(uint16_t conn_handle, const char *where)
{
    struct ble_gap_conn_desc desc;
    memset(&desc, 0, sizeof(desc));
    if (ble_gap_conn_find(conn_handle, &desc) != 0) {
        ESP_LOGW(TAG, "Conn params unavailable at %s", where ? where : "unknown");
        return;
    }
    ESP_LOGI(TAG,
             "Conn params (%s): interval=%.2fms latency=%u supervision=%.2fms",
             where ? where : "unknown",
             (float)desc.conn_itvl * 1.25f,
             desc.conn_latency,
             (float)desc.supervision_timeout * 10.0f);
}

static void app_ble_request_low_latency_conn_params(uint16_t conn_handle, const char *where)
{
    struct ble_gap_upd_params params = {
        .itvl_min = 6,   /* 7.5 ms */
        .itvl_max = 12,  /* 15 ms */
        .latency = 0,
        .supervision_timeout = 200, /* 2 s */
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    int rc = ble_gap_update_params(conn_handle, &params);
    if (rc == 0 || rc == BLE_HS_EALREADY || rc == BLE_HS_EBUSY) {
        ESP_LOGI(TAG, "Conn param update requested at %s (rc=%d)", where ? where : "unknown", rc);
    } else {
        ESP_LOGW(TAG, "Conn param update failed at %s rc=%d", where ? where : "unknown", rc);
    }
}

static int app_ble_access_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    (void)attr_handle;
    (void)conn_handle;

    app_ble_char_t which = (app_ble_char_t)(intptr_t)arg;

    if (which == APP_CHAR_EVENT || which == APP_CHAR_AUDIO) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    uint8_t buf[APP_BLE_PACKET_MAX];
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }

    if (ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (which == APP_CHAR_CONTROL) {
        if (len < 1) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (buf[0] != APP_CTRL_HELLO && !s_app_ready) {
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        switch (buf[0]) {
            case APP_CTRL_HELLO: {
                if (!s_security_ready) {
                    return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
                }
                if (len < 2) {
                    ESP_LOGW(TAG, "Reject hello: missing protocol version");
                    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                }
                if (buf[1] != APP_PROTOCOL_VERSION) {
                    ESP_LOGW(TAG,
                             "Reject hello: protocol mismatch app=%u fw=%u",
                             (unsigned)buf[1],
                             (unsigned)APP_PROTOCOL_VERSION);
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                }
                bool already_ready = s_app_ready;
                app_ble_mark_app_ready();
                if (already_ready) {
                    /* Re-sync state when app re-attaches to an already-ready link. */
                    app_ble_link_notify_state(s_last_notified_state);
                }
                break;
            }
            case APP_CTRL_START_CAPTURE:
                if (!s_audio_notify_enabled) {
                    ESP_LOGW(TAG, "Reject start capture: audio notifications disabled");
                    return BLE_ATT_ERR_CCCD_IMPROPERLY_CONFIGURED;
                }
                if (s_callbacks.on_control_start_capture) {
                    s_callbacks.on_control_start_capture();
                }
                break;
            case APP_CTRL_STOP_CAPTURE:
                if (s_callbacks.on_control_stop_capture) {
                    s_callbacks.on_control_stop_capture();
                }
                break;
            case APP_CTRL_PING:
                break;
            case APP_CTRL_SET_CONFIG:
                break;
            case APP_CTRL_AGENT_ACTIVITY:
                if (len < 2) {
                    ESP_LOGW(TAG, "Reject agent activity: missing status");
                    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                }
                if (s_callbacks.on_control_agent_activity) {
                    s_callbacks.on_control_agent_activity(buf[1]);
                }
                break;
            default:
                ESP_LOGW(TAG, "Unknown control opcode: 0x%02X", buf[0]);
                return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        return 0;
    }

    if (which == APP_CHAR_RESULT) {
        if (!s_app_ready) {
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        if (len < 4) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint16_t session_id = app_le16_read(&buf[0]);
        uint8_t status = buf[2];
        uint8_t text_len = buf[3];
        if (text_len > APP_MAX_TRANSCRIPT_BYTES) {
            text_len = APP_MAX_TRANSCRIPT_BYTES;
        }
        if ((uint16_t)(4 + text_len) > len) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        char text[APP_MAX_TRANSCRIPT_BYTES + 1];
        memset(text, 0, sizeof(text));
        memcpy(text, &buf[4], text_len);

        if (s_callbacks.on_result_text) {
            s_callbacks.on_result_text(session_id, status, text);
        }
        return 0;
    }

    if (which == APP_CHAR_AUDIO_DOWNLINK) {
        if (!s_app_ready) {
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        if (len < sizeof(app_audio_downlink_packet_header_t)) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        app_audio_downlink_packet_header_t header;
        memcpy(&header, buf, sizeof(header));
        if (header.payload_len > APP_AUDIO_DOWNLINK_MAX_PAYLOAD_BYTES) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if ((uint16_t)(sizeof(header) + header.payload_len) > len) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        if (s_callbacks.on_audio_downlink_packet && header.payload_len > 0) {
            s_callbacks.on_audio_downlink_packet(header.session_id,
                                                 header.seq,
                                                 header.flags,
                                                 header.codec,
                                                 header.pcm_sample_count,
                                                 header.codec_param0,
                                                 header.codec_param1,
                                                 &buf[sizeof(header)],
                                                 header.payload_len);
        }
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &s_control_uuid.u,
                .access_cb = app_ble_access_cb,
                .arg = (void *)APP_CHAR_CONTROL,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &s_event_uuid.u,
                .access_cb = app_ble_access_cb,
                .arg = (void *)APP_CHAR_EVENT,
                .val_handle = &s_event_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &s_audio_uuid.u,
                .access_cb = app_ble_access_cb,
                .arg = (void *)APP_CHAR_AUDIO,
                .val_handle = &s_audio_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &s_result_uuid.u,
                .access_cb = app_ble_access_cb,
                .arg = (void *)APP_CHAR_RESULT,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &s_audio_downlink_uuid.u,
                .access_cb = app_ble_access_cb,
                .arg = (void *)APP_CHAR_AUDIO_DOWNLINK,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {0},
        },
    },
    {0},
};

static int app_ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                app_ble_reset_link_state();
                s_transport_connected = true;
                s_conn_handle = event->connect.conn_handle;
                uint16_t mtu = ble_att_mtu(s_conn_handle);
                s_audio_payload_max = (mtu > 3) ? (uint16_t)(mtu - 3) : APP_BLE_DEFAULT_NOTIFY_PAYLOAD_MAX;
                ESP_LOGI(TAG, "Connected. conn_handle=%u mtu=%u", s_conn_handle, mtu);
                app_ble_log_conn_params(s_conn_handle, "connect");
                app_ble_request_low_latency_conn_params(s_conn_handle, "connect");

                struct ble_gap_conn_desc desc;
                memset(&desc, 0, sizeof(desc));
                bool has_desc = ble_gap_conn_find(s_conn_handle, &desc) == 0;
                bool encrypted = has_desc && desc.sec_state.encrypted;
                bool bonded = has_desc && desc.sec_state.bonded;

                if (encrypted) {
                    ESP_LOGI(TAG, "Link already encrypted (bonded=%d)", bonded ? 1 : 0);
                    app_ble_mark_link_secured();
                } else {
                    s_pairing_in_progress = true;
                    if (s_callbacks.on_pairing_started) {
                        s_callbacks.on_pairing_started();
                    }

                    int rc = ble_gap_security_initiate(s_conn_handle);
                    if (rc == BLE_HS_EALREADY) {
                        ESP_LOGI(TAG, "Security already in progress");
                    } else if (rc != 0) {
                        ESP_LOGW(TAG, "ble_gap_security_initiate failed: %d", rc);
                        app_ble_handle_pairing_failed();
                        app_ble_terminate_connection();
                    }
                }
            } else {
                ESP_LOGW(TAG, "Connect failed status=%d", event->connect.status);
                app_ble_reset_link_state();
                app_ble_advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected. reason=%d", event->disconnect.reason);
            app_ble_reset_link_state();
            if (s_callbacks.on_disconnected) {
                s_callbacks.on_disconnected();
            }
            app_ble_advertise();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_event_val_handle) {
                s_event_notify_enabled = event->subscribe.cur_notify != 0;
                ESP_LOGI(TAG, "Event notify %s", s_event_notify_enabled ? "enabled" : "disabled");
            } else if (event->subscribe.attr_handle == s_audio_val_handle) {
                s_audio_notify_enabled = event->subscribe.cur_notify != 0;
                ESP_LOGI(TAG, "Audio notify %s", s_audio_notify_enabled ? "enabled" : "disabled");
            }
            return 0;

        case BLE_GAP_EVENT_MTU:
            s_audio_payload_max = (event->mtu.value > 3) ? (uint16_t)(event->mtu.value - 3) : APP_BLE_DEFAULT_NOTIFY_PAYLOAD_MAX;
            ESP_LOGI(TAG, "MTU updated to %u", event->mtu.value);
            return 0;

        case BLE_GAP_EVENT_PASSKEY_ACTION: {
            struct ble_sm_io pkey = {0};
            int rc = 0;

            switch (event->passkey.params.action) {
                case BLE_SM_IOACT_DISP:
                    if (s_active_passkey == 0) {
                        s_active_passkey = app_ble_generate_passkey();
                    }
                    pkey.action = BLE_SM_IOACT_DISP;
                    pkey.passkey = s_active_passkey;
                    rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
                    ESP_LOGI(TAG, "Display passkey: %" PRIu32 " (rc=%d)", pkey.passkey, rc);
                    app_ble_notify_passkey(pkey.passkey);
                    if (s_callbacks.on_pairing_passkey) {
                        s_callbacks.on_pairing_passkey(pkey.passkey);
                    }
                    return 0;

                case BLE_SM_IOACT_NUMCMP:
                    pkey.action = BLE_SM_IOACT_NUMCMP;
                    pkey.numcmp_accept = 1;
                    rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
                    ESP_LOGI(TAG, "Numeric comparison accepted (rc=%d)", rc);
                    return 0;

                case BLE_SM_IOACT_INPUT:
                    ESP_LOGW(TAG, "Received INPUT passkey action with display-only IO capabilities");
                    return BLE_HS_EINVAL;

                case BLE_SM_IOACT_NONE:
                case BLE_SM_IOACT_OOB:
                default:
                    return 0;
            }
        }

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.status == 0) {
                ESP_LOGI(TAG, "Link encrypted");
                app_ble_log_conn_params(event->enc_change.conn_handle, "enc_change");
                app_ble_request_low_latency_conn_params(event->enc_change.conn_handle, "enc_change");
                app_ble_mark_link_secured();
            } else {
                ESP_LOGW(TAG, "Encryption failed: %d", event->enc_change.status);
                app_ble_handle_pairing_failed();
                app_ble_terminate_connection();
            }
            return 0;

        case BLE_GAP_EVENT_CONN_UPDATE:
            ESP_LOGI(TAG, "Connection parameters updated status=%d", event->conn_update.status);
            if (event->conn_update.status == 0) {
                app_ble_log_conn_params(event->conn_update.conn_handle, "conn_update");
            }
            return 0;

        default:
            return 0;
    }
}

static void app_ble_reset_link_state(void)
{
    s_transport_connected = false;
    s_security_ready = false;
    s_app_ready = false;
    s_event_notify_enabled = false;
    s_audio_notify_enabled = false;
    s_pairing_in_progress = false;
    s_active_passkey = 0;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_audio_payload_max = APP_BLE_DEFAULT_NOTIFY_PAYLOAD_MAX;
    portENTER_CRITICAL(&s_audio_tx_lock);
    memset(&s_audio_tx_stats, 0, sizeof(s_audio_tx_stats));
    portEXIT_CRITICAL(&s_audio_tx_lock);
}

static void app_ble_mark_link_secured(void)
{
    if (s_security_ready) {
        return;
    }
    s_security_ready = true;
    s_pairing_in_progress = false;
    s_active_passkey = 0;
    if (s_callbacks.on_link_secured) {
        s_callbacks.on_link_secured();
    }
}

static void app_ble_mark_app_ready(void)
{
    if (s_app_ready) {
        return;
    }
    s_pairing_in_progress = false;
    s_app_ready = true;
    if (s_callbacks.on_app_ready) {
        s_callbacks.on_app_ready();
    }
}

static void app_ble_handle_pairing_failed(void)
{
    s_pairing_in_progress = false;
    s_security_ready = false;
    s_app_ready = false;
    s_active_passkey = 0;
    if (s_callbacks.on_pairing_failed) {
        s_callbacks.on_pairing_failed();
    }
}

static void app_ble_terminate_connection(void)
{
    if (s_transport_connected && s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        int rc = ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0) {
            ESP_LOGW(TAG, "ble_gap_terminate failed: %d", rc);
        }
    }
}

static void app_ble_advertise(void)
{
    struct ble_hs_adv_fields adv_fields;
    memset(&adv_fields, 0, sizeof(adv_fields));

    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.tx_pwr_lvl_is_present = 1;
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name = (const uint8_t *)APP_BLE_DEVICE_NAME;
    rsp_fields.name_len = strlen(APP_BLE_DEVICE_NAME);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &adv_params, app_ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising started");
    }
}

static uint32_t app_ble_generate_passkey(void)
{
    uint32_t value = esp_random() % 900000;
    return value + 100000;
}

static void app_ble_notify_passkey(uint32_t passkey)
{
    uint8_t payload[5] = {
        APP_EVT_PAIRING_PASSKEY,
        (uint8_t)(passkey & 0xFF),
        (uint8_t)((passkey >> 8) & 0xFF),
        (uint8_t)((passkey >> 16) & 0xFF),
        (uint8_t)((passkey >> 24) & 0xFF),
    };
    app_ble_notify_raw(s_event_val_handle, payload, sizeof(payload));
}

static void app_ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    app_ble_advertise();
}

static void app_ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "Reset, reason=%d", reason);
}

static void app_ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t app_ble_link_init(const app_ble_callbacks_t *callbacks)
{
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    if (s_initialized) {
        return ESP_OK;
    }

    memset(&s_callbacks, 0, sizeof(s_callbacks));
    if (callbacks) {
        s_callbacks = *callbacks;
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.reset_cb = app_ble_on_reset;
    ble_hs_cfg.sync_cb = app_ble_on_sync;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(APP_BLE_DEVICE_NAME);

    ble_store_config_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return ESP_FAIL;
    }

    nimble_port_freertos_init(app_ble_host_task);
    s_initialized = true;
    return ESP_OK;
}

bool app_ble_link_is_connected(void)
{
    return s_app_ready;
}

bool app_ble_link_audio_notify_ready(void)
{
    return s_transport_connected && s_security_ready && s_app_ready && s_audio_notify_enabled;
}

void app_ble_link_begin_audio_session(uint16_t session_id)
{
    portENTER_CRITICAL(&s_audio_tx_lock);
    memset(&s_audio_tx_stats, 0, sizeof(s_audio_tx_stats));
    s_audio_tx_stats.session_id = session_id;
    portEXIT_CRITICAL(&s_audio_tx_lock);
}

void app_ble_link_get_audio_tx_stats(uint16_t session_id, app_ble_audio_tx_stats_t *stats_out)
{
    if (!stats_out) {
        return;
    }
    memset(stats_out, 0, sizeof(*stats_out));
    portENTER_CRITICAL(&s_audio_tx_lock);
    if (s_audio_tx_stats.session_id == session_id) {
        *stats_out = s_audio_tx_stats;
    } else {
        stats_out->session_id = session_id;
    }
    portEXIT_CRITICAL(&s_audio_tx_lock);
}

void app_ble_link_notify_state(app_device_state_t state)
{
    s_last_notified_state = state;
    if (!s_event_notify_enabled) {
        return;
    }
    uint8_t payload[4] = {APP_EVT_STATE_CHANGED, (uint8_t)state, 0, 0};
    app_ble_notify_raw(s_event_val_handle, payload, sizeof(payload));
}

void app_ble_link_notify_level(uint16_t level)
{
    if (!s_event_notify_enabled) {
        return;
    }
    uint8_t payload[4] = {
        APP_EVT_LEVEL_METER,
        (uint8_t)(level & 0xFF),
        (uint8_t)((level >> 8) & 0xFF),
        0,
    };
    app_ble_notify_raw(s_event_val_handle, payload, sizeof(payload));
}

void app_ble_link_notify_capture_stopped(uint16_t session_id, app_capture_stop_reason_t reason)
{
    if (!s_event_notify_enabled) {
        return;
    }
    uint8_t payload[6] = {APP_EVT_CAPTURE_STOPPED, (uint8_t)reason, 0, 0, 0, 0};
    app_le16_write(&payload[2], session_id);
    app_ble_notify_raw(s_event_val_handle, payload, sizeof(payload));
}

void app_ble_link_notify_error(uint8_t error_code)
{
    if (!s_event_notify_enabled) {
        return;
    }
    uint8_t payload[4] = {APP_EVT_ERROR, error_code, 0, 0};
    app_ble_notify_raw(s_event_val_handle, payload, sizeof(payload));
}

void app_ble_link_notify_agent_status(uint8_t status, const char *detail)
{
    if (!s_event_notify_enabled) {
        return;
    }

    uint8_t payload[64];
    payload[0] = APP_EVT_AGENT_STATUS;
    payload[1] = status;
    uint8_t text_len = 0;
    if (detail) {
        size_t n = strlen(detail);
        if (n > 60) {
            n = 60;
        }
        text_len = (uint8_t)n;
        memcpy(&payload[3], detail, text_len);
    }
    payload[2] = text_len;
    app_ble_notify_raw(s_event_val_handle, payload, (uint16_t)(3 + text_len));
}

void app_ble_link_notify_audio_downlink_ready(uint8_t credits)
{
    if (!s_event_notify_enabled) {
        return;
    }
    uint8_t payload[3] = {APP_EVT_AUDIO_DOWNLINK_READY, credits, 0};
    app_ble_notify_raw(s_event_val_handle, payload, sizeof(payload));
}

void app_ble_link_notify_audio_downlink_done(uint16_t session_id, uint8_t status)
{
    if (!s_event_notify_enabled) {
        return;
    }
    uint8_t payload[4] = {
        APP_EVT_AUDIO_DOWNLINK_DONE,
        (uint8_t)(session_id & 0xFF),
        (uint8_t)((session_id >> 8) & 0xFF),
        status,
    };
    app_ble_notify_raw(s_event_val_handle, payload, sizeof(payload));
}

void app_ble_link_send_audio_packet(uint16_t session_id,
                                    uint16_t seq,
                                    uint16_t capture_elapsed_ms,
                                    uint8_t codec,
                                    uint8_t flags,
                                    const uint8_t *payload,
                                    uint16_t payload_len)
{
    if (payload_len > APP_AUDIO_MAX_ADPCM_BYTES) {
        payload_len = APP_AUDIO_MAX_ADPCM_BYTES;
    }

    uint8_t packet[sizeof(app_audio_packet_header_t) + APP_AUDIO_MAX_ADPCM_BYTES];
    app_audio_packet_header_t header = {
        .session_id = session_id,
        .seq = seq,
        .capture_elapsed_ms = capture_elapsed_ms,
        .codec = codec,
        .flags = flags,
        .payload_len = payload_len,
    };

    memcpy(packet, &header, sizeof(header));
    if (payload_len > 0 && payload) {
        memcpy(packet + sizeof(header), payload, payload_len);
    }

    if (!app_ble_link_audio_notify_ready()) {
        if (payload_len > 0) {
            portENTER_CRITICAL(&s_audio_tx_lock);
            if (s_audio_tx_stats.session_id != session_id) {
                memset(&s_audio_tx_stats, 0, sizeof(s_audio_tx_stats));
                s_audio_tx_stats.session_id = session_id;
            }
            s_audio_tx_stats.frames_notify_fail++;
            portEXIT_CRITICAL(&s_audio_tx_lock);
        }
        return;
    }

    uint16_t packet_len = sizeof(header) + payload_len;
    uint8_t retries = 0;
    int rc = app_ble_send_fragmented(s_audio_val_handle, packet, packet_len, &retries);
    if (payload_len > 0) {
        portENTER_CRITICAL(&s_audio_tx_lock);
        if (s_audio_tx_stats.session_id != session_id) {
            memset(&s_audio_tx_stats, 0, sizeof(s_audio_tx_stats));
            s_audio_tx_stats.session_id = session_id;
        }
        s_audio_tx_stats.notify_retry_count += retries;
        if (rc == 0) {
            s_audio_tx_stats.frames_notify_ok++;
        } else {
            s_audio_tx_stats.frames_notify_fail++;
        }
        portEXIT_CRITICAL(&s_audio_tx_lock);
    }
    if (rc != 0) {
        ESP_LOGW(TAG, "Audio notify failed: session=%u seq=%u cap_ms=%u rc=%d retries=%u",
                 session_id,
                 seq,
                 capture_elapsed_ms,
                 rc,
                 retries);
    }
}
