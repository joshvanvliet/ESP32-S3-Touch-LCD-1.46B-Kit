# Power management and concurrent workload validation

The measurements below describe the initial power pass, before the subsequent
TE synchronization fix. Current rendering uses core 1, two 8240-byte staging
buffers, scan-ordered presentation and a released CPU lock during TE waits; see
[rendering details](RENDERING_PERFORMANCE.md). The earlier CPU percentages and
clock residency are historical, not measurements of that newer configuration.

This pass preserves 412 × 412 RGB565, display brightness, the 60 FPS animation
target, audio formats/gain, Bluetooth connection parameters, and task affinity.
It reduces unnecessary activity and permits lower clocks between useful work.
It does not establish a battery-life percentage: the board's battery ADC measures
voltage, not current, and no current meter was available.

## Changes

- ESP-IDF dynamic frequency scaling uses a maximum of 240 MHz and a minimum of
  80 MHz. The peripheral bus remains at 80 MHz. Automatic light sleep stays off,
  so ongoing audio DMA and the connected BLE link are not clock-gated.
- A CPU-frequency lock spans each rendered frame. Allowing clock changes between
  every short DMA fragment increased submission time from about 11 to 13 ms.
  The frame-scoped lock removes that repeated switching cost and releases at
  frame completion, including when a display submission fails.
- Bluetooth modem sleep uses the main crystal and lets the controller sleep
  between BLE events. Connection interval, security, MTU, and audio payloads
  are unchanged. This is separate from system light sleep.
- LVGL reads `esp_timer_get_time()` directly. Its former 2 ms tick timer is no
  longer created, removing 500 periodic timer callbacks per second.
- The main task waits for the next frame/LVGL deadline, with a 5 ms maximum for
  periodic state/credit servicing. Queued state events notify the task immediately.
  Notifications remain pending while it renders, closing the queue-to-sleep race.
  It no longer replays missed 1 ms service polls after a long render.
- The PCM5101 speaker driver allocates only its TX channel. Its unused RX channel
  had an unconnected input pin but still allocated buffers and ran DMA. Removing
  it recovered approximately 6.7 KB of internal RAM. The real microphone remains
  on its separate I2S1 controller. WakeNet's memory guard was not weakened.
- Some of that recovered RAM funds two 5768-byte display buffers (seven full
  rows each). The usual animation now needs about 12–13 DMA chunks per frame
  instead of 18. Total staging memory is 11536 bytes, 3296 more than before.

The power policy follows ESP-IDF's
[power-management documentation](https://docs.espressif.com/projects/esp-idf/en/v5.3.5/esp32s3/api-reference/system/power_management.html).
Existing sdkconfig files need these settings applied explicitly; defaults affect
new configurations. Disabling PM or the custom LVGL clock keeps the corresponding
fixed-clock/periodic-tick fallback available.

## What was measured

Measurements were taken on the USB-connected ESP32-S3 using task runtime counters
clocked by ESP_TIMER, display profiling, and power-mode residency counters. These
counters add overhead and are disabled in the release configuration.

The initial steady baseline held 60.002 FPS over 45 seconds. Core 0 was about 72%
busy and core 1 mostly idle. Rendering and WakeNet dominate active CPU time; this
pass should not be described as a large reduction in total CPU utilization.
The timer task's observed share fell from about 1.13% to 0.51% of one core when
the periodic LVGL tick was removed (the remaining timers still run).

The final 45-second steady capture, after audio stress and WakeNet rearm, measured
60.008 FPS with zero skipped frames and zero logged errors. Comparable baseline
and final CPU measurements were:

| Measurement | Baseline | Final configuration |
| --- | ---: | ---: |
| Main task, percent of one core | 50.77% | 47.52% |
| Core 0 busy (100% minus idle task) | 71.91% | 70.92% |
| Core 1 busy | 0.27% | 0.30% |
| CPU clock time at 80 MHz | 0% | 13.07% |
| Mean frame submission wall time | 10.99 ms | 10.71 ms |

The main task's active share fell about 6.4% relative to baseline. Total busy time
fell less, because Bluetooth modem-sleep management itself uses additional CPU
time (controller task about 0.45% → 3.33% of one core). Clock residency and radio
sleep are energy-saving mechanisms, not measured watts or battery-life gains.
Animation and sensor input are live, so small timing differences include workload
variation; the eliminated timer/RX channel and lower chunk count are structural.

An initial PM diagnostic build fell just below WakeNet's 40,000-byte internal
heap guard. It was rejected. Removing the unused speaker RX allocation restored
the memory margin; both initial WakeNet startup and rearm were then exercised.

A same-build transport comparison, with PM/runtime profiling enabled and the
CPU held at 240 MHz, gave these results before audio/BLE initialization:

| Transfer (30 frames, including packing and final DMA completion) | 4120 bytes × 2 | 5768 bytes × 2 |
| --- | ---: | ---: |
| Full 412 × 412 | 16.16 ms/frame | 14.01 ms/frame |
| 252 × 136 rectangle | 3.32 ms/frame | 2.96 ms/frame |

These are transfer benchmarks, not end-to-end full-screen rendering rates.
Do not directly compare their small timing differences with the earlier
non-PM build as if instrumentation and configuration were identical.

The optional stress test exercises three eight-second playback sessions:

1. ADPCM playback concurrently with WakeNet microphone processing and rendering.
2. ADPCM playback concurrently with real microphone capture/ADPCM encoding and
   rendering; capture restarts after the normal no-speech timeout.
3. Opus playback at 24 kHz, 20 ms frames, and 48 kbit/s, matching the companion
   app's configured bitrate, concurrently with rendering.

In the final stress run, all three sessions completed successfully: 1200 decoded
packets, no sequence gaps, decoder failures, ring overflows, underruns, or I2S
playback failures. Four microphone capture sessions produced 504 audio frames
with no read errors/timeouts; their maximum frame interval was 20–21 ms. There
were no enqueue rejections or observed BLE disconnections, and WakeNet rearmed.
Steady subintervals in each combined workload stayed approximately 60 FPS with
zero skipped frames. Status transitions and reinitialization are excluded, as
described below. Iriun camera inspection showed clean display output.

The encrypted phone BLE connection stays active throughout, including audio
credit/status notifications. Synthetic downlink packets are injected at the
decoder queue; captured audio is counted locally, not sent to a backend. This
does **not** replace an end-to-end phone audio-transfer or perceptual voice test.
The final user check should make several normal voice requests, listen for clean
playback, observe animation during audio, and confirm WakeNet returns afterwards.

Frame-rate claims refer to steady workload intervals. Startup, synchronous audio
reinitialization, and status changes that request a full redraw can still skip
frames; this pass does not claim a hard 16.67 ms deadline through those transitions.
It also does not claim that arbitrary future full-screen scenes will reach 60 FPS.

The normal firmware was rebuilt with all diagnostics disabled and flashed to
COM3. The release reboot confirmed encrypted BLE reconnection, 5768-byte × 2
display buffers, and WakeNet listening. Internal free heap before WakeNet was
51,060 bytes, above its unchanged 40,000-byte guard. No errors were logged in the
30-second release boot capture, and the final Iriun camera check was clean.

## Reproducing the diagnostics

With Python and ffmpeg on PATH, generate the synthetic Opus fixture:

```text
python tests/power/generate_fixture.py
```

Temporarily include `tests/power/hardware_profile.h` and
`tests/power/hardware_stress.h` from `main.c`, and call `power_profile_poll()` and
`power_stress_poll()` in the main loop. Call `power_stress_start()` once after
`app_state_init()` to launch the packet-only feeder task. Lifecycle operations
remain on the main task; packets arrive independently, like actual BLE input.
Feeding them from a refresh-blocked UI loop creates artificial audio starvation.
Enable `CONFIG_APP_FACE_PROFILE`,
`CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS`,
`CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER`, and `CONFIG_PM_PROFILING`.
Record USB serial output for at least 100 seconds after boot. The stress test
starts at 30 seconds and emits `POWER_STRESS` and `Downlink summary` diagnostics.
Check BLE continuity, capture timing/errors, codec errors, underruns, and WakeNet
rearm, not just the reported FPS.

The stress fixture replaces capture callbacks for local measurement. Remove both
headers/calls and disable profiling before producing normal firmware, then flash
and reboot to restore production callbacks. No test includes or calls belong in
the release entry point. The generated tone header is ignored by Git.

Transport correctness checks remain:

```text
python tests/lcd_blit/run.py
python tests/qspi_io/run.py
```
