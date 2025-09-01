# ESP32-S3-Touch-LCD-5 Touch Setup Documentation

## Overview
This document describes the working configuration for the Waveshare ESP32-S3-Touch-LCD-5 board with GT911 capacitive touch controller and LVGL graphics library.

## Hardware Configuration
- **Board**: Waveshare ESP32-S3-Touch-LCD-5
- **Microcontroller**: ESP32-S3
- **Display**: 1024x600 RGB LCD
- **Touch Controller**: GT911 capacitive touch (I2C address: 0x5D)
- **IO Expander**: CH422G (I2C address: 0x24 for config, 0x38 for data)
- **I2C Pins**: SDA=GPIO8, SCL=GPIO9, Frequency=400kHz

## Pin Mapping
```
ESP32-S3 GPIO | Function
---------------|---------
GPIO8         | I2C SDA
GPIO9         | I2C SCL
GPIO3         | LCD VSYNC
GPIO46        | LCD HSYNC
GPIO5         | LCD DE
GPIO7         | LCD PCLK
GPIO4         | Touch INT (output mode)
GPIO14-40    | LCD RGB Data (16-bit)
```

## CH422G IO Expander Control
The CH422G controls various LCD and touch functions:
- **Config Register (0x24)**: Set to 0x01 for output mode
- **Data Register (0x38)**: Controls individual IO pins
- **IO Mapping**:
  - IO0: LCD_BL (Backlight)
  - IO1: TP_RST (Touch Reset)
  - IO2: LCD_RST (LCD Reset)
  - IO3: LCD_CS (LCD Chip Select)

## Touch Controller Setup

### GT911 Reset Sequence (via CH422G)
```c
// Configure CH422G for output mode
i2c_write(0x24, 0x01)

// Pull TP_RST low (reset active)
i2c_write(0x38, 0x2C)  // 0x2E & ~0x02
delay(100ms)

// Pull TP_RST high (reset inactive)
i2c_write(0x38, 0x2E)  // 0x2E | 0x02
delay(200ms)

// Set INT pin high
gpio_set_level(GPIO4, 1)
```

### GT911 Configuration
```c
gt911_config_t config = {
    .i2c_port = I2C_NUM_0,
    .i2c_addr = 0x5D,        // Primary address
    .int_pin = GPIO_NUM_4,   // Touch interrupt pin
    .rst_pin = -1,           // Reset handled by CH422G
    .max_x = 1024,
    .max_y = 600
};
```

## LCD Configuration
- **Resolution**: 1024x600
- **Color Depth**: 16-bit RGB
- **Pixel Clock**: 16MHz
- **Backlight**: Controlled via CH422G IO0

### LCD Reset Sequence
```c
// Configure CH422G for output mode
i2c_write(0x24, 0x01)

// Reset cycle 1-3
for (int i = 0; i < 3; i++) {
    // Pull LCD_RST low
    i2c_write(0x38, 0x16)  // Base value without LCD_RST bit
    delay(500ms)

    // Pull LCD_RST high
    i2c_write(0x38, 0x1E)  // Base value with LCD_RST bit
    delay(300ms)
}
delay(1000ms)  // Final stabilization
```

## LVGL Integration

### Display Driver
```c
// RGB panel configuration
esp_lcd_rgb_panel_config_t panel_config = {
    .timings = {
        .pclk_hz = 16 * 1000 * 1000,
        .h_res = 1024,
        .v_res = 600,
        .hsync_back_porch = 150,
        .hsync_front_porch = 165,
        .hsync_pulse_width = 30,
        .vsync_back_porch = 23,
        .vsync_front_porch = 12,
        .vsync_pulse_width = 2,
    },
    // ... other config
};
```

### Touch Driver
```c
// LVGL touch callback
void gt911_lvgl_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    gt911_touch_data_t touch_data;
    if (gt911_read_touch(gt911_handle, &touch_data) == ESP_OK && touch_data.point_count > 0) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = touch_data.points[0].x;
        data->point.y = touch_data.points[0].y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
```

## Initialization Sequence

1. **Initialize I2C bus** (400kHz, pull-ups enabled)
2. **Reset LCD** via CH422G (3 cycles, 500ms low, 300ms high)
3. **Initialize LCD panel** with RGB configuration
4. **Turn on backlight** via CH422G IO0
5. **Reset GT911** via CH422G (100ms low, 200ms high)
6. **Initialize GT911** (try both I2C addresses 0x5D and 0x14)
7. **Initialize LVGL** with display and touch drivers
8. **Create UI elements** and register touch callbacks

## Troubleshooting

### Touch Not Working
1. Check I2C bus with `i2c_scanner`
2. Verify CH422G reset sequence timing
3. Try alternate GT911 I2C address (0x14)
4. Check GPIO4 configuration (should be output mode)
5. Monitor GT911 status register reads

### Display Issues
1. Verify LCD reset sequence
2. Check RGB timing parameters
3. Ensure backlight is enabled
4. Verify pixel clock frequency

### I2C Communication
1. Check SDA/SCL pin connections
2. Verify pull-up resistors
3. Monitor I2C bus with logic analyzer
4. Check for address conflicts

## Key Files
- `main.c`: Main application and initialization
- `waveshare_rgb_lcd_port.c`: LCD and I2C initialization
- `gt911_touch.c`: GT911 touch driver
- `lvgl_port.c`: LVGL display and touch integration

## Build Configuration
- **PlatformIO**: ESP32-S3-DevKitC-1 board
- **Framework**: ESP-IDF 5.4.1
- **LVGL**: Version 8.4.0
- **Build Flags**: Default ESP-IDF configuration

## Performance Notes
- Touch polling rate: ~30-60Hz
- Display refresh: 60Hz
- I2C bus: 400kHz (stable for both LCD and touch)
- Memory usage: ~45% RAM, ~45% Flash

## Recovery Steps (if touch stops working)

1. **Hard Reset**: Power cycle the board
2. **Check Connections**: Verify I2C and GPIO connections
3. **Rebuild and Upload**: Clean build with `pio run -t clean && pio run -t upload`
4. **Monitor Output**: Use `pio device monitor` to check initialization logs
5. **Test I2C**: Use I2C scanner to verify device presence
6. **Check Timing**: Ensure reset sequences have correct delays

## Version History
- **v1.0**: Initial working configuration with GT911 touch support
- **Date**: August 31, 2025
- **Status**: Fully functional with touch input working
