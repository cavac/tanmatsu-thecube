// Simple 5x7 bitmap font for fast software rendering
// Only includes characters needed for FPS display
// Public domain

#ifndef SIMPLE_FONT_H
#define SIMPLE_FONT_H

#include <stdint.h>

#define FONT_WIDTH 5
#define FONT_HEIGHT 7
#define FONT_SPACING 1

// 5x7 bitmap font data - each character is 7 bytes (one per row)
// Bits are left-to-right: bit 4 = leftmost pixel, bit 0 = rightmost
static const uint8_t font_data[][FONT_HEIGHT] = {
    // '0' (index 0)
    { 0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110 },
    // '1' (index 1)
    { 0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110 },
    // '2' (index 2)
    { 0b01110, 0b10001, 0b00001, 0b00110, 0b01000, 0b10000, 0b11111 },
    // '3' (index 3)
    { 0b01110, 0b10001, 0b00001, 0b00110, 0b00001, 0b10001, 0b01110 },
    // '4' (index 4)
    { 0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010 },
    // '5' (index 5)
    { 0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110 },
    // '6' (index 6)
    { 0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110 },
    // '7' (index 7)
    { 0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000 },
    // '8' (index 8)
    { 0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110 },
    // '9' (index 9)
    { 0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100 },
    // '.' (index 10)
    { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01100, 0b01100 },
    // ' ' (index 11)
    { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 },
    // 'f' (index 12)
    { 0b00110, 0b01000, 0b11100, 0b01000, 0b01000, 0b01000, 0b01000 },
    // 'p' (index 13)
    { 0b00000, 0b00000, 0b11110, 0b10001, 0b11110, 0b10000, 0b10000 },
    // 's' (index 14)
    { 0b00000, 0b00000, 0b01110, 0b10000, 0b01110, 0b00001, 0b11110 },
};

// Map ASCII to font index, returns -1 for unsupported characters
static inline int font_char_index(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c == '.') return 10;
    if (c == ' ') return 11;
    if (c == 'f') return 12;
    if (c == 'p') return 13;
    if (c == 's') return 14;
    return -1;  // Unsupported character
}

// Draw a single character to RGB888 framebuffer (BGR byte order)
// For 270° CCW rotated display: draws in buffer so text appears normal on screen
// screen_x, screen_y = where text should appear on the SCREEN (as user sees it)
// fb_height = 480 (the render area height)
static inline int font_draw_char_270(uint8_t* fb, int fb_stride, int fb_height,
                                      int screen_x, int screen_y, char c,
                                      uint8_t r, uint8_t g, uint8_t b) {
    int idx = font_char_index(c);
    if (idx < 0) return FONT_WIDTH + FONT_SPACING;

    const uint8_t* glyph = font_data[idx];

    // For 270° CCW display: screen(x,y) comes from buffer(fb_height-1-y, x)
    // So to place pixel at screen(x,y), write to buffer(fb_height-1-y, x)

    for (int row = 0; row < FONT_HEIGHT; row++) {
        uint8_t bits = glyph[row];
        for (int col = 0; col < FONT_WIDTH; col++) {
            if (bits & (0b10000 >> col)) {
                // No glyph rotation - coordinate transform handles display rotation
                int glyph_x = col;
                int glyph_y = row;

                // Final screen position
                int sx = screen_x + glyph_x;
                int sy = screen_y + glyph_y;

                // Convert screen coords to buffer coords for 270° display
                int buf_x = fb_height - 1 - sy;
                int buf_y = sx;

                if (buf_x >= 0 && buf_x < fb_height && buf_y >= 0 && buf_y < fb_stride) {
                    int pidx = buf_y * fb_stride * 3 + buf_x * 3;
                    fb[pidx + 0] = b;
                    fb[pidx + 1] = g;
                    fb[pidx + 2] = r;
                }
            }
        }
    }

    return FONT_WIDTH + FONT_SPACING;  // Character width (no rotation)
}

// Draw string for 270° rotated display
static inline void font_draw_string_270(uint8_t* fb, int fb_stride, int fb_height,
                                         int screen_x, int screen_y, const char* str,
                                         uint8_t r, uint8_t g, uint8_t b) {
    while (*str) {
        screen_x += font_draw_char_270(fb, fb_stride, fb_height, screen_x, screen_y, *str, r, g, b);
        str++;
    }
}

#endif // SIMPLE_FONT_H
