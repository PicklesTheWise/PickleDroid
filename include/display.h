#ifndef DISPLAY_H
#define DISPLAY_H

#include <lvgl.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>

#ifdef __cplusplus
extern "C" {
#endif

// Display configuration
#define LCD_WIDTH   1024
#define LCD_HEIGHT  600

// RGB LCD pins for ESP32-S3-Touch-LCD-5
#define LCD_PIXEL_CLOCK_HZ     (16 * 1000 * 1000)  // 16MHz pixel clock
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

// RGB Data pins - Corrected according to pinouts.txt
#define PIN_NUM_DATA0   14  // B3 (GPIO14)
#define PIN_NUM_DATA1   38  // B4 (GPIO38) 
#define PIN_NUM_DATA2   18  // B5 (GPIO18)
#define PIN_NUM_DATA3   17  // B6 (GPIO17)
#define PIN_NUM_DATA4   10  // B7 (GPIO10)
#define PIN_NUM_DATA5   39  // G2 (GPIO39)
#define PIN_NUM_DATA6   0   // G3 (GPIO0)
#define PIN_NUM_DATA7   45  // G4 (GPIO45)
#define PIN_NUM_DATA8   48  // G5 (GPIO48)
#define PIN_NUM_DATA9   47  // G6 (GPIO47)
#define PIN_NUM_DATA10  21  // G7 (GPIO21)
#define PIN_NUM_DATA11  1   // R3 (GPIO1)
#define PIN_NUM_DATA12  2   // R4 (GPIO2)
#define PIN_NUM_DATA13  42  // R5 (GPIO42)
#define PIN_NUM_DATA14  41  // R6 (GPIO41)
#define PIN_NUM_DATA15  40  // R7 (GPIO40)

// RGB Control pins  
#define PIN_NUM_PCLK    7   // PCLK (GPIO7)
#define PIN_NUM_CS      -1  // Not used
#define PIN_NUM_DC      -1  // Not used
#define PIN_NUM_RST     -1  // Controlled via CH422G
#define PIN_NUM_BK_LIGHT -1 // Controlled via CH422G

// RGB Sync pins - Corrected according to pinouts.txt  
#define PIN_NUM_HSYNC   46  // HSYNC (GPIO46)
#define PIN_NUM_VSYNC   3   // VSYNC (GPIO3)
#define PIN_NUM_DE      5   // DE (GPIO5) - Corrected

// I2C pins for CH422G - Corrected according to pinouts.txt
#define I2C_SDA     8   // TP_SDA (GPIO8)
#define I2C_SCL     9   // TP_SCL (GPIO9)
#define I2C_FREQ    400000

// CH422G I2C addresses
#define CH422G_CONFIG_ADDR  0x24
#define CH422G_DATA_ADDR    0x38

// RGB LCD timing configuration based on ESP32-S3-Touch-LCD-5 documentation
#define LCD_H_RES                   1024
#define LCD_V_RES                   600
// Critical anti-drift configuration from documentation
#define LCD_HSYNC_PULSE_WIDTH       30    // Enhanced from 10
#define LCD_HSYNC_BACK_PORCH        150   // Anti-drift value from docs
#define LCD_HSYNC_FRONT_PORCH       165   // Anti-drift value from docs  
#define LCD_VSYNC_PULSE_WIDTH       2     // Datasheet stable value
#define LCD_VSYNC_BACK_PORCH        23    // Datasheet stable value
#define LCD_VSYNC_FRONT_PORCH       12    // Datasheet stable value
// Bounce buffer to prevent display drift
#define LCD_RGB_BOUNCE_BUFFER_HEIGHT 10

/**
 * @brief Initialize the RGB LCD display
 * @return true if successful, false otherwise
 */
bool display_init(void);

/**
 * @brief LCD reset sequence via CH422G
 */
void lcd_reset_sequence(void);

/**
 * @brief Initialize RGB LCD panel
 */
void init_rgb_lcd(void);

/**
 * @brief LVGL display flush callback
 * @param disp_drv Display driver
 * @param area Area to flush
 * @param color_p Color data
 */
void display_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p);

/**
 * @brief Get the LCD panel handle
 * @return Panel handle
 */
esp_lcd_panel_handle_t* display_get_panel_handle(void);

/**
 * @brief Get display width
 * @return Display width in pixels
 */
uint16_t display_get_width(void);

/**
 * @brief Get display height
 * @return Display height in pixels
 */
uint16_t display_get_height(void);

#ifdef __cplusplus
}
#endif

#endif // DISPLAY_H
