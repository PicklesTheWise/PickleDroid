// Minimal main leveraging port abstraction (no legacy duplication)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "waveshare_rgb_lcd_port.h"

static const char *TAG = "MAIN";

static void heartbeat_task(void *arg) {
    int counter = 0;
    while (1) {
        ESP_LOGI(TAG, "Heartbeat %d", ++counter);
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Init RGB LCD via port layer");
    ESP_ERROR_CHECK(waveshare_esp32_s3_rgb_lcd_init());
    wavesahre_rgb_lcd_bl_on();

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_make(0x00,0x40,0x80), 0);
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 180, 70);
    lv_obj_center(btn);
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "Touch Test");
    lv_obj_center(label);

    xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 1, NULL);
}
