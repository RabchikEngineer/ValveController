#include "ui_render.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "Render";

static uint8_t s_fb[SCREEN_WIDTH * PAGES];  // 1024 bytes
static uint8_t s_dirty_pages = 0xFF;        // битовая маска dirty pages
static void *s_panel_handle = NULL;

// Простейший 5x7 font (ASCII 32..126), каждый символ 5 байт (вертикальные колонки)
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x5F,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14}, // #
    {0x24,0x2A,0x7F,0x2A,0x12}, // $
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x05,0x03,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00}, // )
    {0x14,0x08,0x3E,0x08,0x14}, // *
    {0x08,0x08,0x3E,0x08,0x08}, // +
    {0x00,0x50,0x30,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x60,0x60,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02}, // /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x36,0x36,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00}, // ;
    {0x08,0x14,0x22,0x41,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14}, // =
    {0x00,0x41,0x22,0x14,0x08}, // >
    {0x02,0x01,0x51,0x09,0x06}, // ?
    {0x32,0x49,0x79,0x41,0x3E}, // @
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x0C,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x3F,0x40,0x38,0x40,0x3F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x07,0x08,0x70,0x08,0x07}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x00,0x7F,0x41,0x41,0x00}, // [
    {0x02,0x04,0x08,0x10,0x20}, // backslash
    {0x00,0x41,0x41,0x7F,0x00}, // ]
    {0x04,0x02,0x01,0x02,0x04}, // ^
    {0x40,0x40,0x40,0x40,0x40}, // _
};





void render_init(void *lcd_panel_handle)
{   
    bool mirror_x = 1, mirror_y = 1;
    s_panel_handle = lcd_panel_handle;
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel_handle, mirror_x, mirror_y));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel_handle, true));
    render_clear();
    render_flush();
    ESP_LOGI(TAG, "Display init completed, mirror: (x: %d, y: %d)", mirror_x, mirror_y);
}

void render_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
    s_dirty_pages = 0xFF;
}

void render_text(uint8_t page, uint8_t col, const char* str,bool invert)
{
    if (page >= PAGES) return;
    uint8_t x = col;
    for (const char *p = str; *p && x < SCREEN_WIDTH - 5; p++, x += 6) {
        uint8_t ch = (uint8_t)*p;
        if (ch < 32 || ch > 126) ch = 32;
        if (ch > 95) ch -= 32; // lowercase to uppercase
        const uint8_t *glyph = font5x7[ch - 32];
        for (int i = 0; i < 5; i++) {
            uint8_t data = glyph[i];
            s_fb[page * SCREEN_WIDTH + x + i] = invert ? ~data : data;
        }
        s_fb[page * SCREEN_WIDTH + x + 5] = invert ? 0xFF : 0x00; // spacing
    }
    s_dirty_pages |= (1 << page);
}


#ifndef RENDER_TEXTF_BUF_SZ
#define RENDER_TEXTF_BUF_SZ 64
#endif

// Optional: lets GCC check printf-like format/args at compile time.
void render_textf(uint8_t page, uint8_t col, bool invert, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));  // fmt is 4th arg, "..." starts at 5th

void render_textf(uint8_t page, uint8_t col, bool invert, const char *fmt, ...)
{
    char buf[RENDER_TEXTF_BUF_SZ];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    render_text(page, col, buf, invert);
}


void render_text_large(uint8_t page, uint8_t col, const char *str)
{
    // 2x height (занимает 2 страницы): верхняя половина + нижняя половина
    if (page >= PAGES - 1) return;
    uint8_t x = col;
    for (const char *p = str; *p && x < SCREEN_WIDTH - 10; p++, x += 12) {
        uint8_t ch = (uint8_t)*p;
        if (ch < 32 || ch > 126) ch = 32;
        if (ch > 95) ch -= 32; // lowercase to uppercase
        const uint8_t *glyph = font5x7[ch - 32];
        for (int i = 0; i < 5; i++) {
            uint8_t orig = glyph[i];
            // Растягиваем по вертикали: каждый бит исходного → 2 бита
            uint16_t stretched = 0;
            for (int b = 0; b < 7; b++) {
                if (orig & (1 << b)) stretched |= (3 << (b * 2));
            }
            s_fb[page * SCREEN_WIDTH + x + i*2]     = (uint8_t)(stretched & 0xFF);
            s_fb[page * SCREEN_WIDTH + x + i*2 + 1] = (uint8_t)(stretched & 0xFF);
            s_fb[(page+1) * SCREEN_WIDTH + x + i*2]     = (uint8_t)((stretched >> 8) & 0xFF);
            s_fb[(page+1) * SCREEN_WIDTH + x + i*2 + 1] = (uint8_t)((stretched >> 8) & 0xFF);
        }
    }
    s_dirty_pages |= (1 << page) | (1 << (page + 1));
}

void render_flush(void)
{
    if (!s_panel_handle || !s_dirty_pages) return;

    for (int p = 0; p < PAGES; p++) {
        if (s_dirty_pages & (1 << p)) {
            int y_start = p * 8;
            int y_end   = y_start + 8;
            esp_lcd_panel_draw_bitmap(s_panel_handle, 0, y_start, SCREEN_WIDTH, y_end,
                                      &s_fb[p * SCREEN_WIDTH]);
        }
    }
    s_dirty_pages = 0;
    // ESP_LOGI(TAG,"Render completed");
}


// static void oled_draw_checkerboard(esp_lcd_panel_handle_t panel, int size)
// {
//     static uint8_t fb[OLED_WIDTH * OLED_HEIGHT / 8]; // 1024 bytes for 128x64, 1bpp [web:209]

//     // Fill framebuffer with a simple pattern: alternating 8x8 blocks
//     for (int y = 0; y < OLED_HEIGHT; y++) {
//         for (int x = 0; x < OLED_WIDTH; x++) {
//             bool on = (((x / size) ^ (y / size)) & 1);
//             int byte_index = x + (y / 8) * OLED_WIDTH;   // SSD1306 page layout
//             uint8_t bit = 1 << (y & 7);
//             if (on) fb[byte_index] |= bit;
//             else    fb[byte_index] &= (uint8_t)~bit;
//         }
//     }

//     ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, fb));
// }



// static void oled_draw_color(esp_lcd_panel_handle_t panel, bool color) {

//     const int arr_size = OLED_WIDTH * OLED_HEIGHT / 8;

//     uint8_t fb[arr_size];

//     uint8_t value_to_fill = 0xFF * color;

//     // ESP_LOGI(TAG, "%x", value_to_fill);

//     // for (int i = 0; i < arr_size; i++) {
        
//     //     fb[i] = value_to_fill;

//     // }

//     memset(fb, value_to_fill, sizeof(fb));

//     ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, fb));
    
// }