#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "waveshare_rgb_lcd_port.h"
#include "gt911_touch.h"

static const char *TAG = "MAIN";
static gt911_handle_t *gt911_handle = NULL;
static lv_indev_t *touch_indev = NULL;
static lv_obj_t *btn_label = NULL;
static int touch_count = 0;

/**
 * @brief GT911 reset sequence
 */
void gt911_reset_sequence(void)
{
    // Configure CH422G to output mode
    uint8_t config_cmd = 0x01;
    esp_err_t ret = i2c_master_write_to_device(0, 0x24, &config_cmd, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CH422G: %s", esp_err_to_name(ret));
        return;
    }
    
    // Reset sequence: 0x2C -> delay -> GPIO4 low -> delay -> 0x2E -> delay
    uint8_t data_cmd = 0x2C;
    i2c_master_write_to_device(0, 0x38, &data_cmd, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Configure GPIO4 as output and pull low
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_4),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_4, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    data_cmd = 0x2E;
    i2c_master_write_to_device(0, 0x38, &data_cmd, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(200));
}

/**
 * @brief LVGL touch input callback
 */
void gt911_lvgl_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static gt911_touch_point_t last_touch = {0};
    gt911_touch_data_t touch_data;
    
    if (gt911_handle) {
        esp_err_t ret = gt911_read_touch(gt911_handle, &touch_data);
        ESP_LOGI(TAG, "gt911_read_touch returned %d, point_count=%d", ret, (int)touch_data.point_count);
        if (ret == ESP_OK && touch_data.point_count > 0) {
            ESP_LOGI(TAG, "TOUCH DETECTED! x=%d, y=%d, size=%d", 
                     touch_data.points[0].x, touch_data.points[0].y, touch_data.points[0].size);
            
            data->state = LV_INDEV_STATE_PRESSED;
            data->point.x = touch_data.points[0].x;
            data->point.y = touch_data.points[0].y;
            
            last_touch = touch_data.points[0];
        } else {
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "gt911_read_touch failed: %d", ret);
            }
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {
        ESP_LOGW(TAG, "gt911_handle is NULL in gt911_lvgl_read");
        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = last_touch.x;
        data->point.y = last_touch.y;
    }
}

/**
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
    ESP_LOGI(TAG, "Initializing PickleDroid Display...");
    
    // Initialize the RGB LCD
    ESP_LOGI(TAG, "Initializing RGB LCD...");
    esp_err_t lcd_ret = waveshare_esp32_s3_rgb_lcd_init();
    if (lcd_ret != ESP_OK) {
        ESP_LOGE(TAG, "RGB LCD initialization failed: %s", esp_err_to_name(lcd_ret));
        return;
    }
    ESP_LOGI(TAG, "RGB LCD initialized successfully");

    ESP_LOGI(TAG, "Initializing GT911 touch controller...");
    
    // Perform GT911 reset sequence
    gt911_reset_sequence();
    
    // Configure GT911
    gt911_config_t gt911_config = {
        .i2c_port = I2C_NUM_0,
        .i2c_addr = GT911_I2C_ADDR_DEFAULT,  // Use 0x5D based on our previous testing
        .int_pin = GPIO_NUM_4,
        .rst_pin = -1,  // We handle reset manually via CH422G
        .max_x = 1024,
        .max_y = 600
    };
    
    esp_err_t touch_ret = gt911_init(&gt911_config, &gt911_handle);
    if (touch_ret == ESP_OK) {
        ESP_LOGI(TAG, "GT911 touch controller initialized successfully");
        
        // Register GT911 as LVGL input device
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = gt911_lvgl_read;
        touch_indev = lv_indev_drv_register(&indev_drv);
        
        ESP_LOGI(TAG, "GT911 registered with LVGL as input device");
    } else {
        ESP_LOGW(TAG, "GT911 initialization failed: %s", esp_err_to_name(touch_ret));
        ESP_LOGW(TAG, "Continuing without touch functionality");
    }

    // Create a simple UI
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_make(0x00,0x40,0x80), 0);
    
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
    if (touch_ret == ESP_OK) {
        lv_label_set_text(status_label, "GT911 Touch Ready - Try touching the button!");
    } else {
        lv_label_set_text(status_label, "Touch controller failed - Check serial output");
    }
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    
    ESP_LOGI(TAG, "=== Initialization Complete ===");
}
