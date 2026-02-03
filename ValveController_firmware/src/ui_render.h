#pragma once

#include <stdint.h>
#include <stdbool.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define PAGES         SCREEN_HEIGHT/8   // 64/8

void render_init(void *lcd_panel_handle);
void render_clear(void);
void render_text(uint8_t page, uint8_t col, const char *str, bool invert);
void render_text_large(uint8_t page, uint8_t col, const char *str); // 2x height
void render_flush(void);  // send on display only dirty pages
