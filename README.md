# Waveshare ESP32-S3 Touch 1.46 Starter

A standalone ESP-IDF starter project for the Waveshare ESP32-S3 Touch 1.46 board.

This project is intended as a hardware bring-up baseline you can zip and publish as a separate repo.
It initializes and demonstrates:

- Display + touch (LVGL UI)
- Battery ADC measurement
- RTC (PCF85063)
- IMU (QMI8658)
- Speaker output (startup chirp + beep button)
- Microphone speech front-end (WakeNet + Multinet + live level logs)

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

## What You Should See

In serial logs:

- `App/Speech: I2S mic slot: RIGHT`
- `App/Speech: Ready`
- `App/Speech: MIC level avg=... peak=...`

On screen:

- Live battery / RTC / IMU values
- `Touch Test Button` counter increments when touched
- `Play Speaker Beep` button emits a short test tone
- Backlight slider changes panel brightness

## Project Layout

- `main/main.c`:
  Board bring-up and main loop.
- `main/starter_ui.c`:
  Demo UI and peripheral test controls.
- `main/MIC_Driver/MIC_Speech.c`:
  Mic pipeline + periodic level logging.
- `main/Audio_Driver/PCM5101.c`:
  Speaker init + `Audio_Play_Test_Tone()` helper.

## Make Your Own App

See [APP_DEVELOPMENT.md](./APP_DEVELOPMENT.md) for recommended extension points.
