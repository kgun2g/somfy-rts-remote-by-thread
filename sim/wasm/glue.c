/* glue.c — 웹 시뮬레이터 ↔ 펌웨어 oled_ui.c 연결(emscripten).
 *  IDF/ssd1306/FreeRTOS stub 구현(no-op) + JS 가 호출할 export 함수. */
#include "oled_ui.h"
#include "ssd1306.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <emscripten.h>
#include <stdint.h>
#include <stdbool.h>

/* ── 펌웨어가 참조하는 외부 심볼 ── */
bool g_rf_ready = true;

/* ── IDF / ssd1306 stub 구현(no-op) ── */
int64_t esp_timer_get_time(void){ return (int64_t)(emscripten_get_now() * 1000.0); }
void vTaskDelay(TickType_t t){ (void)t; }
BaseType_t xTaskCreate(void(*fn)(void*), const char* n, unsigned s, void* a, unsigned p, TaskHandle_t* o){
    (void)fn;(void)n;(void)s;(void)a;(void)p; if(o) *o=(void*)1; return pdPASS; }
void vTaskDelete(TaskHandle_t t){ (void)t; }
SemaphoreHandle_t xSemaphoreCreateMutex(void){ return (void*)1; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t){ (void)s;(void)t; return pdTRUE; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t s){ (void)s; return pdTRUE; }
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t d, const uint8_t* b, size_t l, int t){ (void)d;(void)b;(void)l;(void)t; return ESP_OK; }
esp_err_t i2c_master_probe(i2c_master_bus_handle_t b, uint16_t a, int t){ (void)b;(void)a;(void)t; return ESP_OK; }
void ssd1306_init(SSD1306_t* d, int w, int h){ (void)w;(void)h; if(d) d->_address=0x3C; }
void ssd1306_clear_screen(SSD1306_t* d, bool i){ (void)d;(void)i; }
void ssd1306_contrast(SSD1306_t* d, int c){ (void)d;(void)c; }
void ssd1306_display_image(SSD1306_t* d, int p, int s, const uint8_t* dat, int w){ (void)d;(void)p;(void)s;(void)dat;(void)w; }
void i2c_master_init(SSD1306_t* d, int sda, int scl, int r){ (void)d;(void)sda;(void)scl;(void)r; }
void i2c_device_add(SSD1306_t* d, i2c_port_t port, int r, int a){ (void)d;(void)port;(void)r;(void)a; }

/* ── oled_ui.c 의 sim hook ── */
extern const uint8_t* oled_sim_fb(void);
extern int oled_sim_panel_w(void);
extern int oled_sim_panel_h(void);
extern void oled_sim_render(oled_ui_ctx_t* ctx);

static oled_ui_ctx_t g_ctx;

EMSCRIPTEN_KEEPALIVE void sim_init(void){
    oled_ui_init(&g_ctx);
    oled_ui_set_freq(&g_ctx, 447.675f);
    oled_ui_set_blind(&g_ctx, 0);
}
EMSCRIPTEN_KEEPALIVE void sim_action(int a){ oled_ui_notify_action_start(&g_ctx, (oled_action_t)a); }
EMSCRIPTEN_KEEPALIVE void sim_action_end(void){ oled_ui_notify_action_end(&g_ctx); }
EMSCRIPTEN_KEEPALIVE void sim_select(int idx){ oled_ui_set_blind(&g_ctx, idx); }
EMSCRIPTEN_KEEPALIVE void sim_freq(float f){ oled_ui_set_freq(&g_ctx, f); }
EMSCRIPTEN_KEEPALIVE void sim_tick(void){ oled_sim_render(&g_ctx); }
EMSCRIPTEN_KEEPALIVE const uint8_t* sim_fb(void){ return oled_sim_fb(); }
EMSCRIPTEN_KEEPALIVE int sim_pw(void){ return oled_sim_panel_w(); }
EMSCRIPTEN_KEEPALIVE int sim_ph(void){ return oled_sim_panel_h(); }

int main(void){ return 0; }
