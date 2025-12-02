#ifndef RENDERER_H
#define RENDERER_H

#ifdef __cplusplus
extern "C" {
#endif

void renderer_init(void);

// Render frame directly to framebuffer
// framebuffer: pointer to RGB888 pixel data (can be offset into a larger buffer)
// stride: bytes per row in the framebuffer (e.g., 800*3 for 800-pixel wide display)
// frame_number: animation frame counter
void renderer_render_frame(unsigned char* framebuffer, int stride, int frame_number);

#ifdef __cplusplus
}
#endif

#endif
