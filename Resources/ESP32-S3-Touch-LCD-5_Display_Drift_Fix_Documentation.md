# ESP32-S3-Touch-LCD-5 Display Drift Fix Documentation

## Problem Summary
The ESP32-S3-Touch-LCD-5 (1024×600 RGB LCD) exhibited persistent horizontal display drift/centering issues where:
- Display content would shift horizontally after each reset/power cycle
- Button appeared at the "very bottom of the screen" 
- Screen display was "not centered properly"
- Drift direction would change unpredictably (sometimes left, sometimes right)

## Root Cause Analysis
The drift was caused by a combination of factors:
1. **Aggressive 21MHz pixel clock** creating timing instability margins
2. **Missing bounce buffer** allowing frame timing jitter to accumulate
3. **Insufficient LCD reset sequence** leaving residual timing state in LCD controller
4. **Suboptimal timing values** not accounting for real-world hardware characteristics

## Solution Overview
The fix was derived from analyzing Arduino demo code which contained production-tested optimizations specifically designed to prevent display drift.

## Final Working Configuration

### Critical Parameters
```c
// Pixel Clock - Reduced from 21MHz to 16MHz for stability
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ      (16 * 1000 * 1000)

// Bounce Buffer - Increased from 0 to 10 lines for anti-drift
#define CONFIG_EXAMPLE_LCD_RGB_BOUNCE_BUFFER_HEIGHT 10

// Horizontal Timing - Conservative values between datasheet and aggressive compensation
.hsync_back_porch = 150,   // Was 145 (datasheet) or 160 (compensation attempt)
.hsync_front_porch = 165,  // Was 170 (datasheet) or 155 (compensation attempt)
.hsync_pulse_width = 30,   // Unchanged

// Vertical Timing - Datasheet defaults (stable)
.vsync_back_porch = 23,
.vsync_front_porch = 12,
.vsync_pulse_width = 2,
```

### Enhanced LCD Reset Sequence
```c
// Multi-cycle reset via CH422G I/O expander
void waveshare_rgb_lcd_reset() {
    // 3 reset cycles with extended delays
    for (int cycle = 0; cycle < 3; cycle++) {
        // Reset active (LCD_RST low via CH422G IO3)
        write_buf = 0x16; // Without LCD_RST bit
        i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, timeout);
        esp_rom_delay_us(500 * 1000); // 500ms reset hold
        
        // Reset inactive (LCD_RST high)
        write_buf = 0x1E; // With LCD_RST bit (0x16 + 0x08)
        i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, timeout);
        esp_rom_delay_us(300 * 1000); // 300ms between cycles
    }
    esp_rom_delay_us(1000 * 1000); // 1 second final stabilization
}
```

## Hardware Configuration
- **Board**: ESP32-S3-DevKitC-1 with Waveshare ESP32-S3-Touch-LCD-5
- **Display**: 1024×600 RGB LCD with ST7262 controller
- **I/O Expander**: CH422G for LCD reset and backlight control
- **Framework**: ESP-IDF 5.4.1 via PlatformIO
- **LVGL**: Version 8.4.0

## Key Insights from Arduino Demo Analysis

### 1. Pixel Clock Discovery
Arduino demos revealed two different pixel clock configurations:
- **ColorBar Demo**: 16MHz (stability focused)
- **LVGL Demo**: 21MHz (performance focused)

The 16MHz configuration from the ColorBar demo was specifically optimized for display stability.

### 2. Bounce Buffer Anti-Drift Feature
Arduino code explicitly mentioned bounce buffer configuration "to avoid screen drift":
```cpp
// From Arduino waveshare_lcd_port.cpp
bus->configRGB_BounceBufferSize(EXAMPLE_LCD_RGB_BOUNCE_BUFFER_SIZE);
// Comment: "Set bounce buffer to avoid screen drift"
```

### 3. Proper CH422G Reset Sequence
Arduino implementation showed proper I/O expander usage:
```cpp
esp_expander::CH422G *expander = new esp_expander::CH422G(...);
expander->init();
expander->begin();
lcd->reset();
lcd->begin();
```

## CH422G I/O Expander Details
```
I2C Addresses:
- Configuration: 0x24
- Data: 0x38

Pin Mapping:
- EXIO1 (IO1, 0x02): TP_RST (Touch Reset)
- EXIO2 (IO2, 0x04): LCD_BL (Backlight Control) 
- EXIO3 (IO3, 0x08): LCD_RST (LCD Reset)
- EXIO4 (IO4, 0x10): SD_CS (SD Card Chip Select)

Control Values:
- Base value: 0x16 (without LCD_RST)
- With LCD_RST: 0x1E (0x16 + 0x08)
- Backlight ON: 0x1E
- Backlight OFF: 0x1A
```

## Troubleshooting Timeline
1. **Initial Issue**: Display drift with datasheet timing (145/170/30, 21MHz)
2. **Timing Adjustments**: Various horizontal timing compensations attempted
3. **Reset Investigation**: Added CH422G-based LCD reset functionality
4. **Arduino Analysis**: Discovered 16MHz pixel clock and bounce buffer importance
5. **Solution**: Combined 16MHz clock + 10-line bounce buffer + enhanced reset

## Performance Impact
- **Pixel Clock**: 21MHz → 16MHz (24% reduction, improved stability margins)
- **Memory**: +10KB bounce buffer (1024×10×2 bytes for RGB565)
- **Boot Time**: +3 seconds for comprehensive LCD reset sequence
- **Stability**: Eliminated timing drift completely

## File Locations
```
src/waveshare_rgb_lcd_port.h - Pixel clock and timing configuration
src/lvgl_port.h - Bounce buffer configuration  
src/waveshare_rgb_lcd_port.c - LCD reset implementation and timing values
src/main.c - Application entry point
```

## Verification Steps
1. **Build**: `pio run` - should compile without errors
2. **Flash**: `pio run -t upload` - firmware ~454KB
3. **Test**: Power cycle multiple times, verify display remains centered
4. **Validation**: Button should appear properly centered, no horizontal drift

## Alternative Configurations Tested (Failed)
- 21MHz with datasheet timing (145/170/30) - **Drift present**
- 21MHz with aggressive compensation (160/155/30) - **Drift direction changed**
- 21MHz with moderate compensation (150/165/30) - **Some improvement but still drift**
- Enhanced reset only with 21MHz - **Partial improvement but drift remained**

## Future Considerations
- **Touch Re-enabling**: When esp_lcd_touch component available in PlatformIO
- **Performance Tuning**: Could experiment with 18-19MHz as middle ground
- **Memory Optimization**: Bounce buffer size could be fine-tuned if memory constrained
- **Temperature Testing**: Verify stability across temperature ranges

## References
- **Successful Config**: Arduino ColorBar demo (16MHz, 10-line bounce buffer)
- **Hardware**: ESP32-S3-Touch-LCD-5 schematics and pinout documentation
- **CH422G**: I/O expander datasheet and Arduino implementation examples
- **ESP-IDF**: RGB LCD documentation and timing parameter explanations

## Final Notes
This configuration represents a production-ready solution that balances display stability with performance. The key insight was that reference implementations (Arduino demos) often contain real-world optimizations that pure datasheet specifications may not capture.

**Status**: ✅ RESOLVED - Display properly centered and stable across resets

---
*Document created: August 31, 2025*  
*Last verified: ESP-IDF 5.4.1, PlatformIO Core 6.11.0*
