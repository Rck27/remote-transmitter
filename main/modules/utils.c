#include "iot_button.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

typedef struct {
    button_cb_t callback;
    void *usr_data;
    int64_t last_call_time;
    uint32_t debounce_ms;
} debounced_btn_cb_t;

button_handle_t init_btn(gpio_num_t gpio, button_event_t event, button_cb_t cb, void *usr_data) {
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = gpio,
            .active_level = 0,
        },
    };

    button_handle_t btn = iot_button_create(&gpio_btn_cfg);
    iot_button_register_cb(btn, event, cb, usr_data);

    return btn;
}

void debounced_btn_callback(void *arg, void *usr_data) {
    debounced_btn_cb_t *btn_data = (debounced_btn_cb_t *)usr_data;
    int64_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds

    if ((current_time - btn_data->last_call_time) >= btn_data->debounce_ms) {
        btn_data->callback(arg, btn_data->usr_data);
        btn_data->last_call_time = current_time;
    }
}

button_handle_t init_debounced_btn(gpio_num_t gpio,
    button_event_t event,
    button_cb_t cb,
    void *usr_data,
    uint32_t debounce_ms) {
    debounced_btn_cb_t *btn_data = malloc(sizeof(debounced_btn_cb_t));
    if (btn_data == NULL) {
        return NULL;
    }

    btn_data->callback = cb;
    btn_data->usr_data = usr_data;
    btn_data->last_call_time = 0;
    btn_data->debounce_ms = debounce_ms;

    return init_btn(gpio, event, debounced_btn_callback, btn_data);
}

float mapValue(float value, float inputMin, float inputMax, float outputMin, float outputMax) {
    if (value < inputMin) return outputMin;
    else if (value > inputMax) return outputMax;

    // Map the input value to the output range
    float inputRange = inputMax - inputMin;
    float outputRange = outputMax - outputMin;
    return ((value - inputMin) * outputRange / inputRange) + outputMin;
}

int mapFloatToInt(float x, float in_min, float in_max, int out_min, int out_max) {
    float result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (result < out_min) result = out_min;
    if (result > out_max) result = out_max;
    return (int)result;
}