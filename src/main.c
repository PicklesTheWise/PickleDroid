// Minimal main leveraging port abstraction (no legacy duplication)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "waveshare_rgb_lcd_port.h"
#include "gt911_touch.h"

static const char *TAG = "MAIN";
static lv_obj_t *btn_label = NULL;
static int touch_count = 0;
static gt911_handle_t *gt911_handle = NULL;
static lv_indev_t *touch_indev = NULL;

// I2C configuration for GT911 touch controller (use correct pins from header)
#define I2C_MASTER_SCL_IO           9      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           8      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              0      /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */

/**
 * @brief Initialize I2C master bus for GT911 touch controller
 */
esp_err_t i2c_master_init_debug(void)
{
    ESP_LOGI(TAG, "Initializing I2C master on bus %d", I2C_MASTER_NUM);
    ESP_LOGI(TAG, "I2C pins - SDA: GPIO%d, SCL: GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully at %d Hz", I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

/**
 * @brief Scan I2C bus for devices
 */
void i2c_scan_bus(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus %d...", I2C_MASTER_NUM);
    
    int devices_found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found on the bus");
    } else {
        ESP_LOGI(TAG, "Found %d I2C device(s)", devices_found);
    }
}

/**
 * @brief Test GT911 communication directly
 */
void test_gt911_direct_communication(void)
{
    ESP_LOGI(TAG, "Testing direct GT911 communication...");
    
    // Test both addresses
    uint8_t addresses[] = {0x14, 0x5D};
    for (int i = 0; i < 2; i++) {
        ESP_LOGI(TAG, "Testing address 0x%02X", addresses[i]);
        
        // Try to read status register
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addresses[i] << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x81, true);  // High byte of status register
        i2c_master_write_byte(cmd, 0x4E, true); // Low byte of status register
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            // Try to read the data
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addresses[i] << 1) | I2C_MASTER_READ, true);
            uint8_t status_data;
            i2c_master_read_byte(cmd, &status_data, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            
            ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "GT911 at 0x%02X status: 0x%02X", addresses[i], status_data);
            } else {
                ESP_LOGW(TAG, "Failed to read from GT911 at 0x%02X: %s", addresses[i], esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "Failed to write to GT911 at 0x%02X: %s", addresses[i], esp_err_to_name(ret));
        }
    }
}

/**
 * @brief Perform proper GT911 reset sequence based on ESP-IDF example
 */
void gt911_proper_reset_sequence(void)
{
    ESP_LOGI(TAG, "Performing proper GT911 reset sequence (ESP-IDF style)");
    
    // Step 1: Configure CH422G to output mode
    uint8_t config_cmd = 0x01;
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &config_cmd, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CH422G: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "CH422G configured to output mode");
    
    // Step 2: Write 0x2C to CH422G data address (reset sequence start)
    uint8_t data_cmd = 0x2C;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &data_cmd, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write 0x2C to CH422G: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Written 0x2C to CH422G data register");
    
    // Step 3: 100ms delay
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Step 4: Configure GPIO4 as output and pull low (TP_IRQ)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_4),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_4, 0);
    ESP_LOGI(TAG, "GPIO4 (TP_IRQ) pulled low");
    
    // Step 5: 100ms delay
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Step 6: Write 0x2E to CH422G data address (reset sequence end)
    data_cmd = 0x2E;
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &data_cmd, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write 0x2E to CH422G: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Written 0x2E to CH422G data register");
    
    // Step 7: 200ms stabilization delay
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "GT911 reset sequence complete");
}

/**
 * @brief Test direct GT911 I2C communication
 */
void test_gt911_direct_i2c(void)
{
    ESP_LOGI(TAG, "Testing direct GT911 I2C communication...");
    
    // Test both addresses
    uint8_t addresses[] = {0x14, 0x5D};
    const char *addr_names[] = {"0x14", "0x5D"};
    
    for (int i = 0; i < 2; i++) {
        ESP_LOGI(TAG, "Testing address %s", addr_names[i]);
        
        // Test basic communication
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addresses[i] << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Address %s responds to basic ping", addr_names[i]);
            
            // Try to read product ID register (0x8140)
            uint8_t product_id[4] = {0};
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addresses[i] << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, 0x81, true);  // High byte
            i2c_master_write_byte(cmd, 0x40, true);  // Low byte
            i2c_master_start(cmd);  // Restart
            i2c_master_write_byte(cmd, (addresses[i] << 1) | I2C_MASTER_READ, true);
            i2c_master_read(cmd, product_id, 3, I2C_MASTER_ACK);
            i2c_master_read_byte(cmd, &product_id[3], I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            
            ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Product ID from %s: %02X %02X %02X %02X ('%c%c%c%c')", 
                         addr_names[i], product_id[0], product_id[1], product_id[2], product_id[3],
                         product_id[0], product_id[1], product_id[2], product_id[3]);
            } else {
                ESP_LOGW(TAG, "Failed to read product ID from %s: %s", addr_names[i], esp_err_to_name(ret));
            }
            
            // Try to read status register (0x814E)
            uint8_t status = 0;
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addresses[i] << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, 0x81, true);  // High byte
            i2c_master_write_byte(cmd, 0x4E, true);  // Low byte
            i2c_master_start(cmd);  // Restart
            i2c_master_write_byte(cmd, (addresses[i] << 1) | I2C_MASTER_READ, true);
            i2c_master_read_byte(cmd, &status, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            
            ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Status register from %s: 0x%02X", addr_names[i], status);
            } else {
                ESP_LOGW(TAG, "Failed to read status from %s: %s", addr_names[i], esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "Address %s does not respond: %s", addr_names[i], esp_err_to_name(ret));
        }
    }
}
void gt911_lvgl_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static gt911_touch_point_t last_touch = {0};
    static uint32_t last_debug_time = 0;
    gt911_touch_data_t touch_data;
    
    // Add periodic debug to show the function is being called
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - last_debug_time > 5000) { // Every 5 seconds
        ESP_LOGI(TAG, "GT911 read callback active, handle=%p", gt911_handle);
        last_debug_time = current_time;
    }
    
    if (gt911_handle) {
        esp_err_t ret = gt911_read_touch(gt911_handle, &touch_data);
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "Touch read successful: count=%d", touch_data.point_count);
            if (touch_data.point_count > 0 && touch_data.points[0].is_pressed) {
                data->state = LV_INDEV_STATE_PRESSED;
                data->point.x = touch_data.points[0].x;
                data->point.y = touch_data.points[0].y;
                last_touch = touch_data.points[0];
                ESP_LOGI(TAG, "TOUCH DETECTED! x=%d, y=%d, size=%d", 
                         touch_data.points[0].x, touch_data.points[0].y, touch_data.points[0].size);
            } else {
                data->state = LV_INDEV_STATE_RELEASED;
                data->point.x = last_touch.x;
                data->point.y = last_touch.y;
            }
        } else {
            ESP_LOGW(TAG, "Failed to read touch data: %s", esp_err_to_name(ret));
            data->state = LV_INDEV_STATE_RELEASED;
            data->point.x = last_touch.x;
            data->point.y = last_touch.y;
        }
    } else {
        ESP_LOGW(TAG, "GT911 handle is NULL");
        data->state = LV_INDEV_STATE_RELEASED;
        data->point.x = last_touch.x;
        data->point.y = last_touch.y;
    }
}

// Button click event handler
static void btn_click_handler(lv_event_t *e)
{
    touch_count++;
    if (btn_label) {
        lv_label_set_text_fmt(btn_label, "Touched %d times!", touch_count);
    }
    ESP_LOGI(TAG, "Button touched! Count: %d", touch_count);
}

static void heartbeat_task(void *arg) {
    int counter = 0;
    while (1) {
        ESP_LOGI(TAG, "Heartbeat %d", ++counter);
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== PickleDroid Display Starting ===");
    
    // Initialize I2C master for GT911 touch controller
    ESP_LOGI(TAG, "Initializing I2C bus 0 for GT911 touch controller...");
    ESP_ERROR_CHECK(i2c_master_init_debug());
    
    // Scan I2C bus to see what devices are present
    i2c_scan_bus();
    
    // Test direct GT911 communication
    test_gt911_direct_communication();
    
    // Initialize RGB LCD via port layer
    ESP_LOGI(TAG, "Initializing RGB LCD via port layer...");
    ESP_ERROR_CHECK(waveshare_esp32_s3_rgb_lcd_init());
    wavesahre_rgb_lcd_bl_on();
    
    // Initialize GT911 touch controller using existing implementation
    ESP_LOGI(TAG, "Initializing GT911 touch controller...");
    
    // Perform proper GT911 reset sequence first
    gt911_proper_reset_sequence();
    
    // Test direct I2C communication after reset
    test_gt911_direct_i2c();
    
    gt911_config_t gt911_config = {
        .i2c_port = I2C_MASTER_NUM,
        .i2c_addr = GT911_I2C_ADDR_DEFAULT,  // Use 0x5D based on detection results
        .int_pin = GPIO_NUM_NC,          // No interrupt pin configured for now
        .rst_pin = GPIO_NUM_NC,          // Reset via CH422G handled in RGB port
        .max_x = 1024,                   // Screen resolution 1024x600
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
        ESP_LOGW(TAG, "Trying alternative address...");
        
        // Try the other address
        gt911_config.i2c_addr = GT911_I2C_ADDR_DEFAULT; // 0x5D
        touch_ret = gt911_init(&gt911_config, &gt911_handle);
        if (touch_ret == ESP_OK) {
            ESP_LOGI(TAG, "GT911 initialized with alternative address");
            
            // Register GT911 as LVGL input device
            static lv_indev_drv_t indev_drv;
            lv_indev_drv_init(&indev_drv);
            indev_drv.type = LV_INDEV_TYPE_POINTER;
            indev_drv.read_cb = gt911_lvgl_read;
            touch_indev = lv_indev_drv_register(&indev_drv);
            
            ESP_LOGI(TAG, "GT911 registered with LVGL as input device");
        } else {
            ESP_LOGW(TAG, "GT911 initialization failed completely: %s", esp_err_to_name(touch_ret));
            ESP_LOGW(TAG, "Continuing without touch functionality");
        }
    }

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_make(0x00,0x40,0x80), 0);
    
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 200, 80);
    lv_obj_center(btn);
    lv_obj_add_event_cb(btn, btn_click_handler, LV_EVENT_CLICKED, NULL);
    
    btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Touch Me!");
    lv_obj_center(btn_label);

    // Create a status label at the top
    lv_obj_t *status_label = lv_label_create(scr);
    if (touch_ret == ESP_OK) {
        lv_label_set_text(status_label, "GT911 Touch Ready - Try touching the button!");
    } else {
        lv_label_set_text(status_label, "Touch controller failed - Check serial output");
    }
    lv_obj_align(status_label, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);

    xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 1, NULL);
    
    ESP_LOGI(TAG, "=== Initialization Complete ===");
}
