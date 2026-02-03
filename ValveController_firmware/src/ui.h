#pragma once

void ui_init(void *lcd_panel_handle, float *current_position_ptr);
void ui_run(void);  // UI main loop (call as task)
