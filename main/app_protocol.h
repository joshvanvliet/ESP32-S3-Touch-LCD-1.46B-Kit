#pragma once

#include <stdbool.h>
#include <stdint.h>

#define APP_GATT_SERVICE_UUID "7f5a0001-5d7f-4f51-9ed4-30f4b6d90000"
#define APP_GATT_CONTROL_UUID "7f5a0002-5d7f-4f51-9ed4-30f4b6d90000"
#define APP_GATT_EVENT_UUID   "7f5a0003-5d7f-4f51-9ed4-30f4b6d90000"
#define APP_GATT_AUDIO_UUID   "7f5a0004-5d7f-4f51-9ed4-30f4b6d90000"
#define APP_GATT_RESULT_UUID  "7f5a0005-5d7f-4f51-9ed4-30f4b6d90000"

#define APP_PROTOCOL_VERSION 0x02

#define APP_AUDIO_CODEC_IMA_ADPCM_16K 0x01
#define APP_AUDIO_CODEC_IMA_ADPCM_12K 0x02
#define APP_AUDIO_CODEC_IMA_ADPCM_8K  0x03

#define APP_MAX_TRANSCRIPT_BYTES 60
#define APP_AUDIO_FRAME_MS 20
#define APP_AUDIO_SAMPLE_RATE_16K 16000
#define APP_AUDIO_SAMPLE_RATE_12K 12000
#define APP_AUDIO_SAMPLE_RATE_8K  8000
#define APP_AUDIO_MAX_SAMPLE_RATE APP_AUDIO_SAMPLE_RATE_16K
#define APP_AUDIO_MAX_PCM_SAMPLES_PER_FRAME ((APP_AUDIO_MAX_SAMPLE_RATE * APP_AUDIO_FRAME_MS) / 1000)
#define APP_AUDIO_MAX_ADPCM_BYTES (APP_AUDIO_MAX_PCM_SAMPLES_PER_FRAME / 2)

typedef enum {
    APP_STATE_UNPAIRED = 0,
    APP_STATE_PAIRING,
    APP_STATE_READY,
    APP_STATE_RECORDING,
    APP_STATE_UPLOADING,
    APP_STATE_RESULT,
    APP_STATE_LINKED,
} app_device_state_t;

typedef enum {
    APP_STOP_REASON_SILENCE = 0,
    APP_STOP_REASON_MANUAL = 1,
    APP_STOP_REASON_MAX_LEN = 2,
    APP_STOP_REASON_NO_SPEECH = 3,
    APP_STOP_REASON_LINK_SLOW = 4,
} app_capture_stop_reason_t;

typedef enum {
    APP_CTRL_HELLO = 0x01,
    APP_CTRL_START_CAPTURE = 0x02,
    APP_CTRL_STOP_CAPTURE = 0x03,
    APP_CTRL_PING = 0x04,
    APP_CTRL_SET_CONFIG = 0x05,
} app_control_opcode_t;

typedef enum {
    APP_EVT_STATE_CHANGED = 0x81,
    APP_EVT_LEVEL_METER = 0x82,
    APP_EVT_CAPTURE_STOPPED = 0x83,
    APP_EVT_ERROR = 0x84,
    APP_EVT_PAIRING_PASSKEY = 0x85,
} app_event_opcode_t;

typedef struct __attribute__((packed)) {
    uint16_t session_id;
    uint16_t seq;
    uint16_t capture_elapsed_ms;
    uint8_t codec;
    uint8_t flags;
    uint16_t payload_len;
} app_audio_packet_header_t;

typedef struct __attribute__((packed)) {
    uint16_t session_id;
    uint8_t status;
    uint8_t text_len;
    char text[APP_MAX_TRANSCRIPT_BYTES];
} app_transcript_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t opcode;
    uint8_t value0;
    uint8_t value1;
    uint8_t value2;
} app_event_payload_t;
