# The Cube - 3D Render Demo for Tanmatsu

## Project Overview

A software 3D renderer demo for the Tanmatsu ESP32-P4 badge. Displays a rotating flat-shaded cube on the 800x480 display.

## Build Instructions

```bash
make build      # Build the project
make flash      # Flash to device
```

## Architecture

- **Pure C implementation** - No C++ dependencies to minimize binary size
- **PSRAM allocation** - Large buffers (framebuffer, z-buffer) allocated from external PSRAM to save internal SRAM
- **Software rasterization** - Triangle rasterization with z-buffer and flat shading

## Key Files

- `main/main.c` - Application entry point, display setup, main loop
- `main/renderer.c` - Pure C 3D renderer with cube geometry
- `main/renderer.h` - Renderer API (renderer_init, renderer_render_frame)

## Renderer Details

The renderer implements:
- 4x4 matrix transformations (lookat, perspective, viewport)
- Barycentric coordinate interpolation
- Z-buffer depth testing
- Flat shading with diffuse lighting
- Backface culling

Cube is rendered at 480x480 pixels, centered on the 800x480 display with black bars on the sides.

## Memory Considerations

This project is tight on internal SRAM. Large buffers must be allocated from PSRAM:
- `cube_buffer` (691KB) - via heap_caps_malloc(..., MALLOC_CAP_SPIRAM)
- `zbuffer` (921KB) - float array for depth testing
- `framebuffer` (691KB) - internal render target

## Credits

- Loosely based on the [tinyrenderer](https://github.com/ssloy/tinyrenderer) project
- Author: Rene 'cavac' Schickbauer
