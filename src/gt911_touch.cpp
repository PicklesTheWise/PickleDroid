#include "gt911_touch.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "GT911";

#define GT911_TIMEOUT_MS 100
#define CH422G_CONFIG_ADDR 0x24
#define CH422G_DATA_ADDR 0x38
#define CH422G_TP_RST_BIT 0x02  // IO1 for TP_RST

/**
 * @brief Write data to GT911 register
 */
static esp_err_t gt911_write_reg(gt911_handle_t *handle, uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t write_buf[len + 2];
    write_buf[0] = (reg >> 8) & 0xFF;  // High byte
    write_buf[1] = reg & 0xFF;         // Low byte
    
    for (size_t i = 0; i < len; i++) {
        write_buf[i + 2] = data[i];
    }
    
    return i2c_master_write_to_device(handle->config.i2c_port, handle->config.i2c_addr, 
                                     write_buf, len + 2, GT911_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Read data from GT911 register
 */
static esp_err_t gt911_read_reg(gt911_handle_t *handle, uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t reg_buf[2];
    reg_buf[0] = (reg >> 8) & 0xFF;  // High byte
    reg_buf[1] = reg & 0xFF;         // Low byte
    
    return i2c_master_write_read_device(handle->config.i2c_port, handle->config.i2c_addr,
                                       reg_buf, 2, data, len, GT911_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t gt911_reset_via_ch422g(gt911_handle_t *handle)
{
    ESP_LOGI(TAG, "Resetting GT911 via CH422G");
    
    // Configure CH422G to output mode
    uint8_t config_cmd = 0x01;
    esp_err_t ret = i2c_master_write_to_device(handle->config.i2c_port, CH422G_CONFIG_ADDR, 
                                              &config_cmd, 1, GT911_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CH422G: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Reset sequence: TP_RST low -> delay -> TP_RST high -> delay
    uint8_t data_cmd;
    
    // Pull TP_RST low (reset active)
    data_cmd = 0x1C; // Base value without TP_RST bit (0x1E & ~0x02)
    ret = i2c_master_write_to_device(handle->config.i2c_port, CH422G_DATA_ADDR, 
                                    &data_cmd, 1, GT911_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to pull TP_RST low: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms reset hold
    
    // Pull TP_RST high (reset inactive)
    data_cmd = 0x1E; // Base value with TP_RST bit
    ret = i2c_master_write_to_device(handle->config.i2c_port, CH422G_DATA_ADDR, 
                                    &data_cmd, 1, GT911_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to pull TP_RST high: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(200)); // 200ms stabilization
    
    ESP_LOGI(TAG, "GT911 reset complete");
    return ESP_OK;
}

esp_err_t gt911_init(const gt911_config_t *config, gt911_handle_t **handle)
{
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing GT911 touch controller");
    
    // Allocate handle
    *handle = (gt911_handle_t*)malloc(sizeof(gt911_handle_t));
    if (!*handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for GT911 handle");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy configuration
    (*handle)->config = *config;
    (*handle)->initialized = false;
    
    // Configure interrupt pin if specified
    if ((*handle)->config.int_pin != GPIO_NUM_NC) {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << (*handle)->config.int_pin);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure interrupt pin: %s", esp_err_to_name(ret));
            free(*handle);
            return ret;
        }
    }
    
    // Reset the touch controller
    esp_err_t ret = gt911_reset_via_ch422g(*handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset GT911: %s", esp_err_to_name(ret));
        free(*handle);
        return ret;
    }
    
    // Try to read product ID to verify communication
    uint8_t pid[4];
    ret = gt911_read_reg(*handle, GT911_REG_PID, pid, 4);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GT911 Product ID: %c%c%c%c", pid[0], pid[1], pid[2], pid[3]);
        (*handle)->initialized = true;
    } else {
        ESP_LOGW(TAG, "Failed to read GT911 Product ID, assuming device is present");
        (*handle)->initialized = true; // Assume it's working for now
    }
    
    ESP_LOGI(TAG, "GT911 initialization complete");
    return ESP_OK;
}

esp_err_t gt911_read_touch(gt911_handle_t *handle, gt911_touch_data_t *touch_data)
{
    if (!handle || !touch_data || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize touch data
    memset(touch_data, 0, sizeof(gt911_touch_data_t));
    
    // Read status register
    uint8_t status;
    esp_err_t ret = gt911_read_reg(handle, GT911_REG_STATUS, &status, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Check if touch data is ready
    if (!(status & 0x80)) {
        return ESP_OK; // No new touch data
    }
    
    // Get number of touch points
    uint8_t point_count = status & 0x0F;
    if (point_count > 5) {
        point_count = 5; // GT911 supports max 5 points
    }
    
    touch_data->point_count = point_count;
    
    // Read touch point data
    for (uint8_t i = 0; i < point_count; i++) {
        uint8_t point_data[8];
        uint16_t point_reg = GT911_REG_TOUCH1 + (i * 8);
        
        ret = gt911_read_reg(handle, point_reg, point_data, 8);
        if (ret != ESP_OK) {
            continue;
        }
        
        // Parse touch point data
        touch_data->points[i].is_pressed = true;
        touch_data->points[i].x = (point_data[1] << 8) | point_data[0];
        touch_data->points[i].y = (point_data[3] << 8) | point_data[2];
        touch_data->points[i].size = (point_data[5] << 8) | point_data[4];
        
        // Simple touch to screen clamping - clamp coordinates to display resolution
        if (touch_data->points[i].x > handle->config.max_x) {
            touch_data->points[i].x = handle->config.max_x;
        }
        if (touch_data->points[i].y > handle->config.max_y) {
            touch_data->points[i].y = handle->config.max_y;
        }
        
        // Also ensure coordinates are not negative (though unlikely with uint16_t)
        // This is more for defensive programming
        if (touch_data->points[i].x < 0) {
            touch_data->points[i].x = 0;
        }
        if (touch_data->points[i].y < 0) {
            touch_data->points[i].y = 0;
        }
    }
    
    // Clear status register to acknowledge
    uint8_t clear_status = 0;
    gt911_write_reg(handle, GT911_REG_STATUS, &clear_status, 1);
    
    return ESP_OK;
}

esp_err_t gt911_deinit(gt911_handle_t *handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Deinitializing GT911");
    free(handle);
    
    return ESP_OK;
}
