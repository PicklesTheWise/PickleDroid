/**
 * ESP Panel Library Configuration
 * 
 * This file configures the ESP_Panel_Library for the Waveshare ESP32-S3-Touch-LCD-5 board
 * with 1024x600 RGB LCD display and CH422G I/O expander.
 */

#pragma once

#include <ESP_Panel_Library.h>

// Display configuration
#define BOARD_ESP32_S3_LCD_EV_BOARD     (1)
#define ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ        (16000000)  // 16MHz pixel clock for stability
#define ESP_PANEL_LCD_RGB_BOUNCE_BUF_SIZE       (LCD_WIDTH * 10)  // 10 lines bounce buffer

// RGB LCD pin definitions for ESP32-S3-Touch-LCD-5
#define ESP_PANEL_LCD_RGB_CLK_HZ        (16 * 1000 * 1000)
#define ESP_PANEL_LCD_RGB_HPW           (30)
#define ESP_PANEL_LCD_RGB_HBP           (150)
#define ESP_PANEL_LCD_RGB_HFP           (165)
#define ESP_PANEL_LCD_RGB_VPW           (2)
#define ESP_PANEL_LCD_RGB_VBP           (23)
#define ESP_PANEL_LCD_RGB_VFP           (12)

// Pin definitions
#define ESP_PANEL_LCD_RGB_PCLK_IO       (7)
#define ESP_PANEL_LCD_RGB_DE_IO         (5)
#define ESP_PANEL_LCD_RGB_VSYNC_IO      (3)
#define ESP_PANEL_LCD_RGB_HSYNC_IO      (46)

// RGB data pins
#define ESP_PANEL_LCD_RGB_DATA0_IO      (14)  // B3
#define ESP_PANEL_LCD_RGB_DATA1_IO      (38)  // B4
#define ESP_PANEL_LCD_RGB_DATA2_IO      (18)  // B5
#define ESP_PANEL_LCD_RGB_DATA3_IO      (17)  // B6
#define ESP_PANEL_LCD_RGB_DATA4_IO      (10)  // B7
#define ESP_PANEL_LCD_RGB_DATA5_IO      (39)  // G2
#define ESP_PANEL_LCD_RGB_DATA6_IO      (0)   // G3
#define ESP_PANEL_LCD_RGB_DATA7_IO      (45)  // G4
#define ESP_PANEL_LCD_RGB_DATA8_IO      (48)  // G5
#define ESP_PANEL_LCD_RGB_DATA9_IO      (47)  // G6
#define ESP_PANEL_LCD_RGB_DATA10_IO     (21)  // G7
#define ESP_PANEL_LCD_RGB_DATA11_IO     (1)   // R3
#define ESP_PANEL_LCD_RGB_DATA12_IO     (2)   // R4
#define ESP_PANEL_LCD_RGB_DATA13_IO     (42)  // R5
#define ESP_PANEL_LCD_RGB_DATA14_IO     (41)  // R6
#define ESP_PANEL_LCD_RGB_DATA15_IO     (40)  // R7

// CH422G I/O Expander configuration
#define ESP_PANEL_EXPANDER_HOST         (ESP_PANEL_HOST_I2C_ID_0)
#define ESP_PANEL_EXPANDER_I2C_ADDRESS  (0x24)
#define ESP_PANEL_EXPANDER_I2C_SCL_IO   (9)
#define ESP_PANEL_EXPANDER_I2C_SDA_IO   (8)

// Touch configuration
#define ESP_PANEL_TOUCH_I2C_ADDRESS     (0x5D)
#define ESP_PANEL_TOUCH_I2C_SCL_IO      (9)
#define ESP_PANEL_TOUCH_I2C_SDA_IO      (8)
#define ESP_PANEL_TOUCH_INT_IO          (4)
