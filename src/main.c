#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "waveshare_rgb_lcd_port.h"
#include "lvgl.h"
#include "gt911_touch.h"

static const char *TAG = "MAIN";
static lv_obj_t *btn_label = NULL;
static lv_obj_t *touch_label = NULL;
static int touch_count = 0;
static gt911_handle_t *gt911_handle = NULL;

/**
 * @brief LVGL touch input callback for GT911
 */
void gt911_lvgl_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static gt911_touch_point_t last_touch = {0};
    gt911_touch_data_t touch_data;

    if (gt911_handle) {
        esp_err_t ret = gt911_read_touch(gt911_handle, &touch_data);
        if (ret == ESP_OK && touch_data.point_count > 0) {
            ESP_LOGI(TAG, "TOUCH DETECTED! x=%d, y=%d, size=%d",
                     touch_data.points[0].x, touch_data.points[0].y, touch_data.points[0].size);

            data->state = LV_INDEV_STATE_PRESSED;
            data->point.x = touch_data.points[0].x;
            data->point.y = touch_data.points[0].y;

            last_touch = touch_data.points[0];

            // Update touch info on screen
            if (touch_label) {
                lv_label_set_text_fmt(touch_label, "Touch: x=%d, y=%d",
                                    touch_data.points[0].x, touch_data.points[0].y);
            }
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
            data->point.x = last_touch.x;
            data->point.y = last_touch.y;
        }
    } else {
        ESP_LOGW(TAG, "gt911_handle is NULL in gt911_lvgl_read");
        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = last_touch.x;
        data->point.y = last_touch.y;
    }
}/**
 * @brief Button click event handler
 */
static void btn_click_handler(lv_event_t *e)
{
    touch_count++;
    if (btn_label) {
        lv_label_set_text_fmt(btn_label, "Touched %d times!", touch_count);
    }
    ESP_LOGI(TAG, "Button touched! Count: %d", touch_count);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-S3 PickleDroid Starting ===");

    // Initialize the Waveshare ESP32-S3 RGB LCD with touch support
    ESP_LOGI(TAG, "Initializing Waveshare ESP32-S3 RGB LCD...");
    esp_err_t ret = waveshare_esp32_s3_rgb_lcd_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "LCD initialized successfully");

    // Turn on the screen backlight
    wavesahre_rgb_lcd_bl_on();
    ESP_LOGI(TAG, "Backlight turned on");

    // Initialize GT911 touch controller
    ESP_LOGI(TAG, "Initializing GT911 touch controller...");
    gt911_config_t gt911_config = {
        .i2c_port = I2C_NUM_0,
        .i2c_addr = GT911_I2C_ADDR_DEFAULT,
        .int_pin = GPIO_NUM_4,
        .rst_pin = -1,
        .max_x = 1024,
        .max_y = 600
    };

    // Add delay to ensure I2C bus is stable
    vTaskDelay(pdMS_TO_TICKS(500));

    ret = gt911_init(&gt911_config, &gt911_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GT911 touch controller initialized successfully");

        // Register GT911 as LVGL input device
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = gt911_lvgl_read;
        lv_indev_t *touch_indev = lv_indev_drv_register(&indev_drv);

        ESP_LOGI(TAG, "GT911 registered with LVGL as input device");
    } else {
        ESP_LOGW(TAG, "GT911 initialization failed: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "Continuing without touch functionality");
    }    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(-1)) {
        ESP_LOGI(TAG, "Creating simple touch demo...");

        // Get the active screen
        lv_obj_t *scr = lv_scr_act();

        // Set a nice background color
        lv_obj_set_style_bg_color(scr, lv_color_make(0x00, 0x40, 0x80), 0);

        // Create a button
        lv_obj_t *btn = lv_btn_create(scr);
        lv_obj_set_size(btn, 200, 80);
        lv_obj_center(btn);
        lv_obj_add_event_cb(btn, btn_click_handler, LV_EVENT_CLICKED, NULL);

        // Create button label
        btn_label = lv_label_create(btn);
        lv_label_set_text(btn_label, "Touch Me!");
        lv_obj_center(btn_label);

        // Create a status label
        lv_obj_t *status_label = lv_label_create(scr);
        if (ret == ESP_OK) {
            lv_label_set_text(status_label, "GT911 Touch Ready - Try touching the button!");
        } else {
            lv_label_set_text(status_label, "Touch controller failed - Check serial output");
        }
        lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 20);
        lv_obj_set_style_text_color(status_label, lv_color_white(), 0);

        // Create touch coordinate display
        touch_label = lv_label_create(scr);
        lv_label_set_text(touch_label, "Touch: x=0, y=0");
        lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 50);
        lv_obj_set_style_text_color(touch_label, lv_color_white(), 0);

        // Release the mutex
        lvgl_port_unlock();
    } else {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock");
    }

    ESP_LOGI(TAG, "=== Initialization Complete ===");
}