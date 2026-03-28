#include "ui_render.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "Render";

static uint8_t s_fb[SCREEN_WIDTH * PAGES];  // 1024 bytes
static uint8_t s_dirty_pages = 0xFF;        // dirty pages
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

static inline bool render_is_in_bounds(int x, int y)
{
    return (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT);
}

static inline void render_mark_dirty_y(int y)
{
    if (y < 0 || y >= SCREEN_HEIGHT) return;
    s_dirty_pages |= (1U << (y / 8));
}

static inline void render_mark_dirty_rect(int x, int y, int w, int h)
{
    if (w <= 0 || h <= 0) return;

    int y0 = y;
    int y1 = y + h - 1;

    if (y0 < 0) y0 = 0;
    if (y1 >= SCREEN_HEIGHT) y1 = SCREEN_HEIGHT - 1;
    if (y0 > y1) return;

    for (int page = y0 / 8; page <= y1 / 8; ++page) {
        s_dirty_pages |= (1U << page);
    }
}


void render_set_pixel(int x, int y, bool on)
{
    if (!render_is_in_bounds(x, y)) return;

    uint16_t idx = (y / 8) * SCREEN_WIDTH + x;
    uint8_t mask = (uint8_t)(1U << (y & 7));

    if (on) {
        s_fb[idx] |= mask;
    } else {
        s_fb[idx] &= (uint8_t)~mask;
    }

    render_mark_dirty_y(y);
}

bool render_get_pixel(int x, int y)
{
    if (!render_is_in_bounds(x, y)) return false;

    uint16_t idx = (y / 8) * SCREEN_WIDTH + x;
    uint8_t mask = (uint8_t)(1U << (y & 7));
    return (s_fb[idx] & mask) != 0;
}

void render_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
    s_dirty_pages = 0xFF;
}

void render_clear_rect(int x1, int y1, int x2, int y2)
{
    if (x2 <= x1 || y2 <= y1) return;

    for (int yy = y1; yy < y2; ++yy) {
        for (int xx = x1; xx < x2; ++xx) {
            render_set_pixel(xx, yy, false);
        }
    }
}

void render_hline(int x1, int y, int x2, bool on)
{
    if (x2 <= x1) return;
    for (int xx = x1; xx < x2; ++xx) {
        render_set_pixel(xx, y, on);
    }
}

void render_vline(int x, int y1, int y2, bool on)
{
    if (y2 <= y1) return;
    for (int yy = y1; yy < y2; ++yy) {
        render_set_pixel(x, yy, on);
    }
}

void render_line(int x0, int y0, int x1, int y1, bool on)
{
    int dx = abs(x1 - x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1) {
        render_set_pixel(x0, y0, on);

        if (x0 == x1 && y0 == y1) break;

        int e2 = err << 1;

        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void render_rect(int x1, int y1, int x2, int y2, bool on)
{
    // if (w <= 0 || h <= 0) return;

    render_hline(x1, y1, x2, on);
    render_hline(x1, y2 - 1, x2, on);
    render_vline(x1, y1, y2, on);
    render_vline(x2 - 1, y1, y2, on);
}

void render_fill_rect(int x, int y, int w, int h, bool on)
{
    if (w <= 0 || h <= 0) return;

    for (int yy = y; yy < y + h; ++yy) {
        render_hline(x, yy, w, on);
    }
}


static void render_char_xy(int x, int y, char ch, bool invert)
{
    uint8_t uch = (uint8_t)ch;

    if (uch < 32 || uch > 126) uch = 32;
    if (uch > 95) uch -= 32;

    const uint8_t *glyph = font5x7[uch - 32];

    for (int col = 0; col < 5; ++col) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; ++row) {
            bool on = ((bits >> row) & 0x01U) != 0;
            if (invert) on = !on;
            render_set_pixel(x + col, y + row, on);
        }
    }

    for (int row = 0; row < 7; ++row) {
        render_set_pixel(x + 5, y + row, invert);
    }

    render_mark_dirty_rect(x, y, 6, 7);
}

void render_text_xy(int x, int y, bool invert, const char *str)
{
    if (!str) return;

    int start_x = x;

    for (const char *p = str; *p; ++p) {
        if (*p == '\n') {
            x = start_x;
            y += 8;
            if (y >= SCREEN_HEIGHT) break;
            continue;
        }

        if (x > SCREEN_WIDTH - 6) break;
        render_char_xy(x, y, *p, invert);
        x += 6;
    }
}

void render_text_page_col(uint8_t page, uint8_t col, bool invert, const char* str)
{
    render_text_xy(col, page * 8, invert, str);
    // if (page >= PAGES) return;
    // uint8_t x = col;
    // for (const char *p = str; *p && x < SCREEN_WIDTH - 4; p++, x += 6) {
    //     uint8_t ch = (uint8_t)*p;
    //     if (ch < 32 || ch > 126) ch = 32;
    //     if (ch > 95) ch -= 32; // lowercase to uppercase
    //     const uint8_t *glyph = font5x7[ch - 32];
    //     for (int i = 0; i < 5; i++) {
    //         uint8_t data = glyph[i];
    //         s_fb[page * SCREEN_WIDTH + x + i] = invert ? ~data : data;
    //     }
    //     s_fb[page * SCREEN_WIDTH + x + 5] = invert ? 0xFF : 0x00; // spacing
    // }
    // s_dirty_pages |= (1 << page);
}

void render_text_large_page_col(uint8_t page, uint8_t col, const char *str)
{
    if (page >= PAGES - 1 || !str) return;

    int x = col;
    int y = page * 8;

    for (const char *p = str; *p && x < SCREEN_WIDTH - 10; ++p, x += 12) {
        uint8_t ch = (uint8_t)*p;
        if (ch < 32 || ch > 126) ch = 32;
        if (ch > 95) ch -= 32;

        const uint8_t *glyph = font5x7[ch - 32];

        for (int col_i = 0; col_i < 5; ++col_i) {
            uint8_t bits = glyph[col_i];
            for (int row = 0; row < 7; ++row) {
                bool on = ((bits >> row) & 0x01U) != 0;
                render_set_pixel(x + col_i * 2 + 0, y + row * 2 + 0, on);
                render_set_pixel(x + col_i * 2 + 1, y + row * 2 + 0, on);
                render_set_pixel(x + col_i * 2 + 0, y + row * 2 + 1, on);
                render_set_pixel(x + col_i * 2 + 1, y + row * 2 + 1, on);
            }
        }
    }
    // // 2x height (занимает 2 страницы): верхняя половина + нижняя половина
    // if (page >= PAGES - 1) return;
    // uint8_t x = col;
    // for (const char *p = str; *p && x < SCREEN_WIDTH - 9; p++, x += 12) {
    //     uint8_t ch = (uint8_t)*p;
    //     if (ch < 32 || ch > 126) ch = 32;
    //     if (ch > 95) ch -= 32; // lowercase to uppercase
    //     const uint8_t *glyph = font5x7[ch - 32];
    //     for (int i = 0; i < 5; i++) {
    //         uint8_t orig = glyph[i];
    //         // Растягиваем по вертикали: каждый бит исходного → 2 бита
    //         uint16_t stretched = 0;
    //         for (int b = 0; b < 7; b++) {
    //             if (orig & (1 << b)) stretched |= (3 << (b * 2));
    //         }
    //         s_fb[page * SCREEN_WIDTH + x + i*2]     = (uint8_t)(stretched & 0xFF);
    //         s_fb[page * SCREEN_WIDTH + x + i*2 + 1] = (uint8_t)(stretched & 0xFF);
    //         s_fb[(page+1) * SCREEN_WIDTH + x + i*2]     = (uint8_t)((stretched >> 8) & 0xFF);
    //         s_fb[(page+1) * SCREEN_WIDTH + x + i*2 + 1] = (uint8_t)((stretched >> 8) & 0xFF);
    //     }
    // }
    // s_dirty_pages |= (1 << page) | (1 << (page + 1));
}

// Optional: lets GCC check printf-like format/args at compile time.
void render_textf_page_col(uint8_t page, uint8_t col, bool invert, bool multiline, const char *fmt, ...)
    __attribute__((format(printf, 5, 6)));  // fmt is 4th arg, "..." starts at 5th

void render_textf_page_col(uint8_t page, uint8_t col, bool invert, bool multiline, const char *fmt, ...)
{
    char buf[RENDER_TEXT_BUF_SZ];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (!multiline) {
        render_text_page_col(page, col, invert, buf);
        return;
    }

    char line[RENDER_TEXT_BUF_SZ];
    int pos=0;


    for (int i=0; i<strlen(buf);i++) {
        if (buf[i]=='\n') {
            line[pos]='\0';
            render_text_page_col(page, col, invert, line);
            page++;
            pos=0;
        } else {
            line[pos]=buf[i];
            pos++;
        }
    }
    line[pos]='\0';
    render_text_page_col(page, col, invert, line);

    
    
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


void renderer_init(void *lcd_panel_handle)
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


