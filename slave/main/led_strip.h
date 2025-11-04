#ifndef __LED_STRIP_H__
#define __LED_STRIP_H__

#include "driver/rmt.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

enum rgb_led_type_t {
    RGB_LED_TYPE_WS2812,
    RGB_LED_TYPE_SK6812,
    RGB_LED_TYPE_APA106
};

struct led_color_t {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct led_strip_t {
    uint32_t led_strip_length;
    rmt_channel_t rmt_channel;
    gpio_num_t gpio;
    struct led_color_t *led_strip_buf_1;
    struct led_color_t *led_strip_buf_2;
    SemaphoreHandle_t access_semaphore;
    bool showing_buf_1;
    enum rgb_led_type_t rgb_led_type;
};

bool led_strip_init(struct led_strip_t *led_strip);
bool led_strip_set_pixel_color(struct led_strip_t *led_strip,
                               uint32_t pixel_num, struct led_color_t *color);
bool led_strip_set_pixel_rgb(struct led_strip_t *led_strip, uint32_t pixel_num,
                             uint8_t red, uint8_t green, uint8_t blue);
bool led_strip_get_pixel_color(struct led_strip_t *led_strip,
                               uint32_t pixel_num, struct led_color_t *color);
bool led_strip_show(struct led_strip_t *led_strip);
bool led_strip_clear(struct led_strip_t *led_strip);

#ifdef __cplusplus
}
#endif

#endif /* __LED_STRIP_H__ */
