# Rendering pipeline

The display uses native 412 × 412 RGB565 at the existing 80 MHz QSPI clock.
The panel's TE signal was measured at 60 rising edges per second on the attached
board. The production animation target remains 60 FPS.

## Implementation

- `app_lcd_qspi_io.c` uses the SPI peripheral's 8-bit command and 24-bit address
  phases. Headers and payloads share one transaction instead of separate
  software transfers. All SPD2010 address commands are still sent.
- Only the final DMA chunk signals completion. The IO adapter honors the
  callback's wake request immediately, avoiding the next FreeRTOS tick. It
  drains completed transactions before reusing descriptors or deleting the IO.
- Two internal DMA buffers overlap packing with the previous transfer. The first
  optimization used 4120 bytes each (five rows), followed by 5768 in the power
  pass. Refresh synchronization now uses 8240 bytes each (ten rows, 16480 total),
  enabled by removing an unused speaker RX DMA allocation. WakeNet's unchanged
  memory guard and audio concurrency are checked on the board.
  The IO adapter also replaces the old ten-entry SPI descriptor pool with one
  persistent descriptor. Large transfers continue under the same CS assertion.
- Dirty regions are fully restored and drawn before presentation. The renderer
  discards stale TE signals and blocks for a fresh rising edge on GPIO18, then
  sends buffer-sized strips ordered by their next scan line. Drawing between
  transfers or finishing one tall region before its neighbor lets the display
  scan catch up with the writer and can cause tearing.
- Touching dirty regions merge only when their bounding box costs no more pixels
  than the two separate regions. This avoids repainting empty corners between
  diagonal shapes. Overlapping regions still produce identical final pixels.
- Contiguous full-width copies use one `memcpy`; opaque spans use LVGL's packed
  RGB565 fill. Geometry, alpha calculations, colors, and resolution are unchanged.
- At the native 60 FPS setting the panel's TE signal sets the cadence, avoiding
  beats between separate software and hardware refresh clocks. Lower configured
  frame rates retain deadline pacing. A missing TE signal times out after about
  34 ms, preserves the previous displayed frame and requests a full retry.
- Failed submissions request a full refresh on the next frame. The startup test
  pattern waits for DMA before reusing or freeing its buffer.

The main/UI task runs on core 1, away from WakeNet and BLE work on core 0. Its
priority rises to 6 only during synchronized presentation and is restored on
success and failure. DMA waits yield the CPU to audio tasks. The CPU-frequency
lock is released while waiting for TE and reacquired before transfer. Audio task
placement and BLE parameters are unchanged. An early 8 KB × 2 buffering
experiment was rejected before the unused speaker RX channel was removed. Omitting
repeated controller address commands was also rejected after camera inspection
showed striping; no command caching remains in the panel driver.

## Initial transport measurements (before TE synchronization)

Measured on the USB-connected ESP32-S3 board on 2026-09-26. Physical output was
inspected using the Nubia NX789J through Iriun Webcam. Instrumented builds used
`CONFIG_APP_FACE_PROFILE=y`; the normal default is off.

The unthrottled transport benchmark sends 30 identical frames, includes packing
and waits for the final DMA completion. It runs before audio/BLE initialization
to isolate display transport capacity; these rates exclude rasterization.

| Workload | Original transport, 8192 × 1 | Optimized transport, 4120 × 2 |
| --- | ---: | ---: |
| Full 412 × 412 frame | 46.00 ms / 21.7 transfers/s | 14.96 ms / 66.9 transfers/s |
| 252 × 136 rectangle | 9.01 ms | 3.07 ms |

The original running application measured 54–55 FPS. The first optimized application
sustained 60.001 FPS over the final 60-second capture with WakeNet listening,
zero skipped frames, and no render errors. Mean CPU-side frame submission time
was 10.70 ms. It is limited by its 60 FPS target; transport-only throughput must
not be presented as application FPS or as the physical panel refresh rate.

The full-frame measurements above compare the original blitter, LCD completion
handling, and stock panel IO against the new transport on the same board. The
benchmark used the same core placement for both runs. Startup, IMU convergence,
BLE reconnection, and WakeNet initialization are excluded from steady-state
application comparisons.

## Refresh-synchronized validation (2026-09-26)

The final instrumented run used two 8240-byte buffers and scan-ordered strips.
Steady windows measured 60.13 FPS before stress, 60.19 with WakeNet plus ADPCM
playback, 60.18 with microphone capture plus ADPCM, 60.21 with Opus playback,
and approximately 60.19 after stress. Rates follow this panel's roughly 16.6 ms
refresh period. These are live workload samples, not a guarantee for every tilt,
full redraw, status transition, or future scene.

Sixty prepared full-screen frames transferred in 12.280 ms on average, with a
12.643 ms maximum, including final DMA completion. This excludes preparing the
test image and waiting for TE; it is not end-to-end full-screen animation FPS.
The deliberately disconnected TE interrupt timed out in 34.476 ms and recovered
after re-enabling it. Normal profiling observed wake delays of tens of microseconds.

All 1200 synthetic downlink packets decoded and played without gaps, underruns,
decoder errors, ring overflows or I2S failures. Four real microphone sessions
produced 504 frames with zero read errors/timeouts and a 22 ms maximum interval.
No enqueue rejection or BLE disconnection was observed, and WakeNet rearmed with
44,256 bytes of internal heap before startup (guard unchanged at 40,000).
The feeder runs independently of the UI task; this checks codec/I2S concurrency
with a connected encrypted BLE link, not the phone/backend voice path itself.

The Iriun camera feed was unavailable during this pass. USB timing and exact
pixel comparisons cannot establish whether a physical tear remains visible;
fast movement and a normal voice request require the final user check.

The normal firmware was rebuilt with profiling disabled and all test hooks
removed, then flashed over COM3. Its 35-second boot check confirmed TE setup,
encrypted phone reconnection and WakeNet listening, with 45,704 bytes free before
WakeNet startup and no runtime errors. The existing legacy-I2C and Bluetooth
light-sleep warnings remain; system light sleep is intentionally disabled.

## Reproducing checks

Run the host tests with Python and GCC on PATH:

```text
python tests/lcd_blit/run.py
python tests/qspi_io/run.py
python tests/face_render/run.py
```

The blit tests verify exact pixel coverage, protected areas, transfer failures,
and deferred DMA buffer ownership with one/two buffers at seven sizes, including
the production 8240-byte setting, plus cost-aware dirty merging. The QSPI tests verify command/address encoding,
parameter bytes, command-only initialization, multi-chunk CS continuation,
descriptor ownership, callback timing, failure recovery, and device lifecycle.
The renderer replay checks all framebuffer and displayed pixel hashes against
the pre-TE reference for 1100 deterministic frames: all moods, blinks, large
motion, protected overlays and full redraws. The new pipeline matches exactly
and transfers 76,033,024 bytes versus the reference's 87,670,032 (13.3% fewer).

For a temporary board benchmark, include `tests/lcd_blit/hardware_benchmark.h`
from `main.c` and call `lcd_transport_benchmark()` after LCD/LVGL initialization
and before `app_state_init()`. It briefly draws a gradient and compares five
staging configurations: four within 8240 bytes and the earlier 11536-byte pair.
Remove the include/call
after profiling; neither is present in the production entry point.

`CONFIG_APP_FACE_PROFILE` logs per-window frame counts and `render_avg_us` in
addition to restore, draw, pack, and wait totals. With TE synchronization,
`render_avg_us` includes the blocking TE wait and final DMA completion; it is
wall time, not CPU utilization. `LCD_TE` reports refresh period, wake delay and
completion relative to the edge. The optional `tests/lcd_te/hardware_sync_test.h`
checks fresh edges, timeout/recovery with the TE interrupt deliberately disabled,
and 60 full-screen moving-stripe transfers. Call `lcd_te_hardware_test()` after
LCD initialization in a temporary profiling build; remove it before release.
This checks timing and failure handling, not optical proof of tear-free output.
