#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include "display.h"
#include "touch_calibration.h"
#include "gt911_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
void setup_ch422g(void);
void gt911_reset_sequence(void);
void create_demo_ui(void);
void process_serial_commands(void);

#ifdef __cplusplus
}
#endif

#endif // MAIN_H
