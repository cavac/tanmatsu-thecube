# The Cube - 3D Render Demo for Tanmatsu

## Project Overview

A software 3D renderer demo for the Tanmatsu ESP32-P4 badge. Displays a rotating textured cube on the 800x480 display using dual-core parallelization.

## Build Instructions

```bash
make build      # Build the project
make flash      # Flash to device
make monitor    # View debug output via USB
```

## Architecture

- **Pure C implementation** - No C++ dependencies to minimize binary size
- **PSRAM allocation** - Large buffers (z-buffer) allocated from external PSRAM to save internal SRAM
- **Software rasterization** - Triangle rasterization with z-buffer and texture mapping
- **Dual-core rendering** - Rasterization split by columns between Core 0 and Core 1
- **Direct framebuffer access** - Renderer writes directly to PAX framebuffer (no intermediate copy)
- **BGR byte order** - Display framebuffer uses BGR888 format (not RGB)

## Key Files

- `main/main.c` - Application entry point, display setup, main loop with performance timing
- `main/renderer.c` - Pure C 3D renderer with cube geometry and dual-core parallelization
- `main/renderer.h` - Renderer API (renderer_init, renderer_render_frame)
- `main/texture_data.h` - Embedded texture (64x64 RGB, wooden crate)
- `main/usb_device.c` - USB debug console initialization for ESP32-P4

## Renderer Details

The renderer implements:
- 4x4 matrix transformations (lookat, perspective, viewport)
- Barycentric coordinate interpolation for UV and depth
- Texture mapping with nearest-neighbor sampling
- Z-buffer depth testing (16-bit)
- Diffuse lighting (modulates texture color)
- Backface culling

Cube is rendered at 480x480 pixels, centered on the 800x480 display with black bars on the sides.

### Dual-Core Parallelization

Rasterization is split by screen columns:
- Core 0 (main task): Columns 0-239
- Core 1 (worker task): Columns 240-479

Synchronization uses FreeRTOS binary semaphores with frame-level sync (one sync per frame, not per triangle) to minimize overhead.

## Performance

Typical frame timing (~19 fps):
- Render: ~52ms (dual-core rasterization)
- Blit: ~0.5ms (DMA to display)

### Optimization History
- Original single-core with float z-buffer: ~9 fps
- After eliminating intermediate copy: ~12 fps
- After dual-core parallelization: ~16 fps
- After 16-bit z-buffer optimization: ~19 fps

## Memory Considerations

This project is tight on internal SRAM. Large buffers must be allocated from PSRAM:
- `zbuffer` (460KB) - int16_t array for depth testing, allocated via heap_caps_malloc(..., MALLOC_CAP_SPIRAM)
- `texture_data` (12KB) - 64x64 RGB texture embedded in flash

The renderer writes directly to the PAX graphics library framebuffer with stride support, eliminating the need for an intermediate render buffer.

## Credits

- Loosely based on the [tinyrenderer](https://github.com/ssloy/tinyrenderer) project
- Author: Rene 'cavac' Schickbauer
