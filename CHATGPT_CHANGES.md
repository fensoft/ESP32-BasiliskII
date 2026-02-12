# Changes Since We Started

Date range: 2026-02-03 (local)

This document summarizes the code changes made during this collaboration, in the order they occurred. It includes work that was later reverted, so the full history is visible.

## Summary

- Added and tuned CPU/video performance optimizations.
- Introduced an opcode cache experiment, then reverted it after it reduced IPS.
- Rebalanced memory placement to prioritize CPU hot tables in internal SRAM.
- Uploaded firmware builds during testing to measure IPS and video performance.

## Detailed Timeline

### 1) CPU and video optimizations (initial pass)

Files:
- `src/basilisk/uae_cpu/newcpu.cpp`
- `src/basilisk/uae_cpu/basilisk_glue.cpp`
- `src/basilisk/uae_cpu/memory.cpp`
- `src/basilisk/uae_cpu/memory.h`
- `src/basilisk/video_esp32.cpp`

Changes:
- Increased CPU batch size for the main emulation loop to reduce overhead per instruction.
- Adjusted opcode table allocation (`cpufunctbl`) to prefer PSRAM so internal SRAM could be used by video buffers.
- Adjusted `mem_banks` allocation to prefer PSRAM as part of the same CPU/video memory balance experiment.
- Video rendering path changes:
  - Forced tile mode as the primary path.
  - Disabled streaming (kept tile-based updates).
  - Increased tile width (fewer tiles, fewer per-tile overheads).
  - Removed tile snapshot compare step for a more direct render path.
  - Preferred internal SRAM for small tile snapshot buffer, and DMA-capable internal SRAM for tile buffers when possible.

Commit:
- `perf: cpu/video tuning experiments and profiling` (pushed to `origin/master`).

### 2) Opcode cache experiment (reverted)

Files created and later removed:
- `src/basilisk/uae_cpu/opcode_cache.h`
- `src/basilisk/uae_cpu/opcode_cache.cpp`

Files modified during experiment:
- `src/basilisk/uae_cpu/newcpu.cpp`
- `src/basilisk/uae_cpu/basilisk_glue.cpp`
- `src/basilisk/uae_cpu/memory.h`
- `src/basilisk/uae_cpu/memory.cpp`

What it did:
- Added a small direct-mapped opcode cache to avoid repeated byteswap for instruction fetches.
- Integrated cache invalidation on RAM writes.

Why reverted:
- Measured IPS dropped vs baseline. The additional cache fill/invalidation cost outweighed benefits.

### 3) CPU-priority memory placement (current state)

Files:
- `src/basilisk/uae_cpu/basilisk_glue.cpp`
- `src/basilisk/uae_cpu/memory.cpp`
- `src/basilisk/uae_cpu/newcpu.h`

Changes:
- `cpufunctbl` now prefers internal SRAM (CPU priority), falling back to PSRAM only if needed.
- `mem_banks` now prefers internal SRAM (CPU priority), falling back to PSRAM only if needed.
- Updated comment in `newcpu.h` to match the new allocation intent.

Effect observed:
- IPS improved meaningfully (approx. 1.39 MIPS average over the first ~55s from boot) versus the prior opcode-cache run.

## Notes

- The video task runs on Core 0, the CPU emulation runs on Core 1. The CPU-priority memory changes help the CPU loop but can pressure internal SRAM that the video path also benefits from.
- Additional warnings during build were unchanged from prior runs (toolchain and framework warnings).

## How to Reproduce Recent Measurement

Commands used:
- Build + upload:
  - `pio run -t upload`
- Serial capture (first 60s approx from boot):
  - `script -q /tmp/pio_perf_cpuprio.log pio device monitor`

Log file:
- `/tmp/pio_perf_cpuprio.log`


### 4) Video row-strip DMA coalescing (in progress)

Files:
- `src/basilisk/video_esp32.cpp`

Changes:
- Added optional row-strip DMA path that coalesces contiguous dirty tiles into horizontal strips.
- Added row-strip buffers (double-buffered) and strided render helpers.
- `renderAndPushDirtyTiles()` now prefers the strip path when buffers are available, with a fallback to per-tile DMA.

Why:
- Reduce the number of `setAddrWindow`/`writePixelsDMA` calls per frame, especially for large dirty regions.

Status:
- Code compiled locally; needs device upload + 60s perf capture to validate render improvements.

### 5) CPU loop batch + tick quantum increase (in progress)

Files:
- `src/basilisk/uae_cpu/newcpu.cpp`
- `src/basilisk/main_esp32.cpp`

Changes:
- Increased `EXEC_BATCH_SIZE` to 8192.
- Increased `emulated_ticks_quantum` to 640000.

Why:
- Reduce tick-check overhead and improve instruction throughput.

Status:
- Needs new 60s boot capture to measure IPS/video impact vs baseline.

### 6) Video tile size increase (in progress)

Files:
- `src/basilisk/video_esp32.cpp`

Changes:
- Increased `TILE_WIDTH` from 80 to 160 (grid now 4x9 = 36 tiles).

Why:
- Reduce per-tile DMA overhead by halving tile count per frame.

Status:
- Needs new 60s boot capture to compare render time vs 80x40 baseline.

Note:
- The 160x40 tile size test regressed render time (~30% worse), so tile size was reverted to 80x40.

### 7) Opcode sampler + hot-opcode instrumentation

Files:
- `src/basilisk/opcode_sampler.cpp`
- `src/basilisk/opcode_sampler.h`
- `src/basilisk/uae_cpu/newcpu.cpp`
- `src/basilisk/main_esp32.cpp`
- `platformio.ini`

Changes:
- Added a lightweight opcode sampler (ring buffer + periodic top-10 report).
- Wired sampling into the hot CPU loop and periodic reporting in `basilisk_loop()`.
- Added build flags to enable/disable sampling and tune sample rate/buffer size.

Notes:
- Used for a 60s boot profile to identify hot opcodes.
- Disabled in perf builds to avoid overhead.

### 8) WiFi auto-connect disabled for perf testing

Files:
- `src/basilisk/boot_gui.cpp`

Changes:
- Added `BOOTGUI_DISABLE_WIFI_AUTOCONNECT` (default on) to skip WiFi auto-connect during perf runs.
- If SSID isn’t available and auto-connect is enabled, the setting is disabled and saved to stop repeated attempts.

### 9) Fast opcode-path experiment (regressed, disabled)

Files:
- `src/basilisk/uae_cpu/newcpu.cpp`
- `platformio.ini`

Changes:
- Added optional fast-path handlers for DBcc + LINK/UNLK (branch/stack frame ops).
- Gated with `FAST_OPCODE_PATH`.

Result:
- Measured IPS dropped vs baseline; feature left in code but disabled by default.

### 10) Direct Bus DMA for Video (guarded by bus type)

Files:
- `src/basilisk/video_esp32.cpp`

Changes:
- Added a direct bus DMA path in `renderAndPushDirtyTiles()`:
  - Uses `M5.Display.getPanel()->getBus()->writeBytes(...)` when the tile buffer is DMA-capable.
  - Falls back to `M5.Display.writePixelsDMA()` when not DMA-capable.
  - DMA waits now use bus-level `wait()` when available.
- Added a guard so direct-bus writes are only used for `bus_spi` and `bus_parallel*`.
  DSI/image-push buses fall back to the original path to avoid a black screen.

Why:
- `writePixelsDMA` performs an internal copy + pixelcopy even when our buffer is already in swap565 format.
- Direct bus DMA can avoid that copy on supported buses.

Notes:
- On ESP32-P4 DSI (the current hardware), direct-bus writes are not supported.

### 11) DSI Direct Framebuffer Path (ESP32-P4)

Files:
- `src/basilisk/video_esp32.cpp`

Changes:
- Added DSI detection via `Panel_DSI` and captured the DSI framebuffer pointer + stride.
- Added an internal-rotation-aware blit path that writes converted tiles directly into the DSI framebuffer (RGB565 nonswapped), then flushes cache via `panel->display(...)`.
- Added a DSI palette variant (nonswapped RGB565) alongside the existing swap565 palette.
- Logged DSI framebuffer details at boot for verification.

Result:
- First 60s render time improved from ~862us average to ~374us average (about 56% faster) vs the previous baseline.
- IPS remained roughly the same (CPU core unaffected).

### 12) Video Frame Skipping (Whole-Frame) + Dirty Merge

Files:
- `src/basilisk/video_esp32.cpp`

Changes:
- Disabled tile-level render budgeting by default (`VIDEO_RENDER_BUDGET_US=0`).
- Added whole-frame skip policy for heavy redraws:
  - `VIDEO_FRAME_MIN_MS` (normal) and `VIDEO_FRAME_MIN_HEAVY_MS` (heavy redraws).
  - `VIDEO_HEAVY_DIRTY_TILES` threshold to select the heavy interval.
- Dirty tiles are now merged into the existing dirty bitmap rather than replacing it.
- Rendered tiles explicitly clear their dirty bit so unrendered tiles persist and are drawn in the next frame.

Why:
- Allows frame “skipping” under heavy redraws without chunked partial updates, reducing bursty memory bandwidth and keeping UI responsive.
