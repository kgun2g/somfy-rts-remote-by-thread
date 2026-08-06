#pragma once
/* sim: esp-idf-ssd1306 라이브러리 대체. 모든 함수 no-op(glue.c 구현).
 *  oled_ui.c 의 _fb_flush 는 ssd1306_display_image 를 호출하지만 sim 에선
 *  s_fb 를 oled_sim_fb() 로 직접 export 하므로 출력은 버려도 된다. */
#include "driver/i2c_master.h"
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    i2c_master_dev_handle_t _i2c_dev_handle;
    i2c_master_bus_handle_t _i2c_bus_handle;
    int _address;
    int _pages;
    int _width;
    int _height;
} SSD1306_t;
void ssd1306_init(SSD1306_t* d, int w, int h);
void ssd1306_clear_screen(SSD1306_t* d, bool invert);
void ssd1306_contrast(SSD1306_t* d, int c);
void ssd1306_display_image(SSD1306_t* d, int page, int seg, const uint8_t* data, int width);
void i2c_master_init(SSD1306_t* d, int sda, int scl, int reset);
void i2c_device_add(SSD1306_t* d, i2c_port_t port, int reset, int addr);
