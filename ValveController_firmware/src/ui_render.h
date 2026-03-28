#pragma once

#include <stdint.h>
#include <stdbool.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define PAGES         SCREEN_HEIGHT/8   // 64/8
#define RENDER_TEXT_BUF_SZ 64

void renderer_init(void *lcd_panel_handle);
void render_clear(void);
void render_flush(void);  // send on display only dirty pages
void render_rect(int x, int y, int w, int h, bool on);
void render_fill_rect(int x, int y, int w, int h, bool on);
void render_clear_rect(int x, int y, int w, int h);
void render_text_xy(int x, int y, bool invert, const char *str);
void render_text_page_col(uint8_t page, uint8_t col, bool invert, const char* str);
void render_text_large_page_col(uint8_t page, uint8_t col, const char *str); // 2x height
void render_textf_page_col(uint8_t page, uint8_t col, bool invert, bool multiline, const char *fmt, ...);
