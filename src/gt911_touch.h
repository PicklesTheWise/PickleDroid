#ifndef GT911_TOUCH_H
#define GT911_TOUCH_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// GT911 I2C addresses
#define GT911_I2C_ADDR_DEFAULT  0x5D
#define GT911_I2C_ADDR_ALT      0x14

// GT911 register addresses
#define GT911_REG_CONFIG        0x8047
#define GT911_REG_PID           0x8140
#define GT911_REG_STATUS        0x814E
#define GT911_REG_TOUCH1        0x8150

// Touch point structure
typedef struct {
    bool is_pressed;
    uint16_t x;
    uint16_t y;
    uint8_t size;
} gt911_touch_point_t;

// Touch data structure
typedef struct {
    uint8_t point_count;
    gt911_touch_point_t points[5]; // GT911 supports up to 5 touch points
} gt911_touch_data_t;

// GT911 configuration structure
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    gpio_num_t int_pin;
    gpio_num_t rst_pin;
    uint16_t max_x;
    uint16_t max_y;
} gt911_config_t;

// GT911 handle
typedef struct {
    gt911_config_t config;
    bool initialized;
} gt911_handle_t;

/**
 * @brief Initialize GT911 touch controller
 * 
 * @param config GT911 configuration
 * @param handle Pointer to store GT911 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gt911_init(const gt911_config_t *config, gt911_handle_t **handle);

/**
 * @brief Read touch data from GT911
 * 
 * @param handle GT911 handle
 * @param touch_data Pointer to store touch data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gt911_read_touch(gt911_handle_t *handle, gt911_touch_data_t *touch_data);

/**
 * @brief Reset GT911 touch controller via CH422G
 * 
 * @param handle GT911 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gt911_reset_via_ch422g(gt911_handle_t *handle);

/**
 * @brief Deinitialize GT911 touch controller
 * 
 * @param handle GT911 handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t gt911_deinit(gt911_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // GT911_TOUCH_H
