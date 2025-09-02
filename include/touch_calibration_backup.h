#ifndef TOUCH_CALIBRATION_H
#define TOUCH_CALIBRATION_H

#include <lvgl.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Touch interrupt pin
#define TOUCH_INT   4   // TP_IRQ (GPIO4)

// Touch controller configuration  
#define GT911_I2C_ADDR_DEFAULT  0x14

/**
 * @brief Initialize touch system
 * @return true if successful, false otherwise
 */
bool touch_init(void);

/**
 * @brief LVGL touch read callback
 * @param indev_drv Input device driver
 * @param data Touch data to fill
 */
void touch_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);

/**
 * @brief Touch interrupt handler
 */
void IRAM_ATTR touch_interrupt_handler(void);

/**
 * @brief Test GT911 direct I2C communication
 * @param addr I2C address to test
 * @return true if communication successful, false otherwise
 */
bool test_gt911_direct_i2c(uint8_t addr);

/**
 * @brief Get calibration status
 * @return true if calibration is active
 */
bool touch_get_calibration_active(void);

/**
 * @brief Check if in calibration mode
 * @return true if in calibration mode
 */
bool touch_is_calibration_mode(void);

#ifdef __cplusplus
}
#endif

#endif // TOUCH_CALIBRATION_H
