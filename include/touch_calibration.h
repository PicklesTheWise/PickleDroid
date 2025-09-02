#ifndef TOUCH_CALIBRATION_H
#define TOUCH_CALIBRATION_H

#include <lvgl.h>

// GT911 Configuration
#define GT911_I2C_ADDR_DEFAULT 0x5D
#define TOUCH_INT 4

// Function declarations
bool touch_init(void);
void touch_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
void IRAM_ATTR touch_interrupt_handler(void);
void handle_touch_interrupt(void);
bool test_gt911_direct_i2c(uint8_t addr);
void test_gt911_touch_direct(void);
void test_gt911_registers(void);

// Simple stub functions for compatibility
bool touch_get_calibration_active(void);
bool touch_get_debug_enabled(void);
void touch_set_debug_enabled(bool enabled);
void start_calibration(void);

#endif // TOUCH_CALIBRATION_H
