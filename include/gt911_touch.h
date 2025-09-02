#ifndef GT911_TOUCH_H
#define GT911_TOUCH_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// GT911 I2C addresses
#define GT911_I2C_ADDR_LOW   0x5D  // When INT pin is low during reset
#define GT911_I2C_ADDR_HIGH  0x14  // When INT pin is high during reset

// GT911 register addresses
#define GT911_REG_PID        0x8140  // Product ID
#define GT911_REG_STATUS     0x814E  // Status register
#define GT911_REG_TOUCH1     0x814F  // First touch point data

// Maximum number of touch points
#define GT911_MAX_TOUCH_POINTS  5

/**
 * @brief GT911 configuration structure
 */
typedef struct {
    i2c_port_t i2c_port;        ///< I2C port number
    uint8_t i2c_addr;           ///< I2C device address
    gpio_num_t int_pin;         ///< Interrupt pin (GPIO_NUM_NC if not used)
    uint16_t max_x;             ///< Maximum X coordinate
    uint16_t max_y;             ///< Maximum Y coordinate
} gt911_config_t;

/**
 * @brief Touch point data structure
 */
typedef struct {
    bool is_pressed;            ///< True if point is pressed
    uint16_t x;                 ///< X coordinate
    uint16_t y;                 ///< Y coordinate
    uint16_t size;              ///< Touch point size/pressure
} gt911_touch_point_t;

/**
 * @brief Touch data structure
 */
typedef struct {
    uint8_t point_count;                            ///< Number of active touch points
    gt911_touch_point_t points[GT911_MAX_TOUCH_POINTS]; ///< Touch point data
} gt911_touch_data_t;

/**
 * @brief GT911 handle structure
 */
typedef struct {
    gt911_config_t config;      ///< Configuration
    bool initialized;           ///< Initialization status
} gt911_handle_t;

/**
 * @brief Initialize GT911 touch controller
 * 
 * @param config Configuration parameters
 * @param handle Pointer to store the handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gt911_init(const gt911_config_t *config, gt911_handle_t **handle);

/**
 * @brief Read touch data from GT911
 * 
 * @param handle GT911 handle
 * @param touch_data Pointer to store touch data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gt911_read_touch(gt911_handle_t *handle, gt911_touch_data_t *touch_data);

/**
 * @brief Reset GT911 via CH422G GPIO expander
 * 
 * @param handle GT911 handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gt911_reset_via_ch422g(gt911_handle_t *handle);

/**
 * @brief Deinitialize GT911 touch controller
 * 
 * @param handle GT911 handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gt911_deinit(gt911_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // GT911_TOUCH_H
