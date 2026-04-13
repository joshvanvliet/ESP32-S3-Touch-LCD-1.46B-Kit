# App Development Guide

This starter separates board bring-up from app logic so you can iterate quickly.

## Where to Customize

- `main/starter_ui.c`
  - Replace the default status dashboard with your own LVGL screens.
  - Keep `starter_ui_update(...)` if you still want periodic sensor refresh.

- `main/main.c`
  - Keep initialization order intact for stable bring-up:
    1. `driver_init()`
    2. `LCD_Init()`
    3. `LVGL_Init()`
    4. `Audio_Init()`
    5. `MIC_Speech_init()`
  - Add your app tasks after these calls.

## Fast Hardware Sanity Checks

- Touch: press `Touch Test Button` and verify counter increments.
- Speaker: press `Play Speaker Beep`.
- Battery/RTC/IMU: watch values update on screen.
- Mic: monitor serial `MIC level avg/peak` logs and speak near mic.

## Microphone Tuning

`main/MIC_Driver/MIC_Speech.c`:

- `MIC_I2S_USE_RIGHT_SLOT`:
  - `1` = right slot
  - `0` = left slot

If wake word is unreliable but mic levels move, test the opposite slot.

## Keeping It Clean

Recommended pattern for your own app:

- Keep board drivers untouched in their folders.
- Add your app module files in `main/` (for example `app_logic.c`, `app_logic.h`).
- Call into your app from `app_main()` after board initialization.

## Optional Extensions

- Enable SD card mount with `SD_Init()` and MP3 playback (`Play_Music(...)`).
- Add BLE/WiFi modules as separate components instead of mixing into board init.
