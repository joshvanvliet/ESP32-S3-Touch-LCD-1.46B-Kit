# ESP Assistant Firmware

ESP-IDF firmware for the Waveshare ESP32-S3 Touch 1.46 board used by the ESP
Assistant Android companion.

The current app firmware provides:

- Display + touch (LVGL UI)
- Battery ADC measurement
- RTC (PCF85063)
- IMU (QMI8658)
- Secure BLE companion link
- Microphone capture streamed to Android as IMA ADPCM
- Transcript/result mirroring from Android
- Agent audio downlink playback over BLE
- Opus 24 kHz and legacy ADPCM downlink decoding
- WakeNet-triggered assistant capture when the Android app is connected

## Quick Start

1. Use a path **without spaces** for building (recommended due some ESP-SR toolchain path quirks).
2. Open ESP-IDF v5.3.x environment.
3. Build and flash:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COM7 flash monitor
```

Replace `COM7` with your port.

## Wake Word

WakeNet is controlled by project Kconfig:

- `CONFIG_APP_WAKEWORD_ENABLED=y`
- `CONFIG_APP_WAKEWORD_RESUME_DELAY_MS=800`
- `CONFIG_APP_WAKEWORD_STOP_TIMEOUT_MS=1200`

When enabled, WakeNet owns the microphone only while the assistant is idle. A
verified wake event stops WakeNet, starts the existing BLE capture path, and then
re-arms after Android/backend thinking and speaker playback finish.

## BLE Audio Downlink

The firmware protocol version is `0x07`. Android can send agent activity control
packets so the firmware pauses WakeNet while the backend is thinking or the
speaker is playing. Downlink audio packets keep the existing 14-byte header.

Supported downlink codecs:

- `0x05`: Opus 24 kHz, 20 ms frames, 480 PCM samples per frame
- `0x04`: IMA ADPCM 24 kHz
- `0x01`: IMA ADPCM 16 kHz
- `0x02`: IMA ADPCM 12 kHz
- `0x03`: IMA ADPCM 8 kHz

## Project Layout

- `main/main.c`:
  Board bring-up and main loop.
- `main/app_state.c`:
  BLE/app state machine, capture lifecycle, wake re-arm policy.
- `main/app_wakeword.c`:
  WakeNet-only idle trigger for assistant capture.
- `main/app_audio_capture.c`:
  Microphone capture + VAD + ADPCM uplink.
- `main/app_audio_downlink.c`:
  Opus/ADPCM downlink decode, buffering, and speaker playback.

## Make Your Own App

See [APP_DEVELOPMENT.md](./APP_DEVELOPMENT.md) for recommended extension points.
