#include <Arduino.h>
#include <Wire.h>
#include "display.h"

// Global variables
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;
static esp_lcd_panel_handle_t panel_handle = NULL;

void lcd_reset_sequence(void) {
    if (Serial) {
        Serial.println("Performing LCD reset sequence...");
    }
    
    // Multi-cycle reset for stability (from documentation)
    for (int cycle = 0; cycle < 3; cycle++) {
        if (Serial) {
            Serial.printf("Reset cycle %d\n", cycle + 1);
        }
        
        // Reset active (LCD_RST low via CH422G IO3)
        Wire.beginTransmission(CH422G_DATA_ADDR);
        Wire.write(0x16);  // Without LCD_RST bit (bit 3 = 0)
        Wire.endTransmission();
        delay(500);  // 500ms reset hold
        
        // Reset inactive (LCD_RST high)
        Wire.beginTransmission(CH422G_DATA_ADDR);
        Wire.write(0x1E);  // With LCD_RST bit (0x16 + 0x08)
        Wire.endTransmission();
        delay(300);  // 300ms between cycles
    }
    
    delay(1000);  // 1 second final stabilization
    if (Serial) {
        Serial.println("LCD reset sequence complete");
    }
}

void init_rgb_lcd(void) {
    if (Serial) {
        Serial.println("=== RGB LCD INITIALIZATION ===");
        Serial.printf("Target resolution: %dx%d\n", LCD_H_RES, LCD_V_RES);
        Serial.printf("Pixel clock: %d Hz\n", LCD_PIXEL_CLOCK_HZ);
        Serial.printf("Free heap before RGB config: %d bytes\n", ESP.getFreeHeap());
    }

    // RGB LCD panel configuration
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .timings = {
            .pclk_hz = LCD_PIXEL_CLOCK_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_pulse_width = LCD_HSYNC_PULSE_WIDTH,
            .hsync_back_porch = LCD_HSYNC_BACK_PORCH,
            .hsync_front_porch = LCD_HSYNC_FRONT_PORCH,
            .vsync_pulse_width = LCD_VSYNC_PULSE_WIDTH,
            .vsync_back_porch = LCD_VSYNC_BACK_PORCH,
            .vsync_front_porch = LCD_VSYNC_FRONT_PORCH,
            .flags = {
                .hsync_idle_low = 0,
                .vsync_idle_low = 0,
                .de_idle_high = 0,
                .pclk_active_neg = 1,
                .pclk_idle_high = 0,
            },
        },
        .data_width = 16, // RGB565
        .sram_trans_align = 4,
        .psram_trans_align = 64,
        .hsync_gpio_num = PIN_NUM_HSYNC,
        .vsync_gpio_num = PIN_NUM_VSYNC,
        .de_gpio_num = PIN_NUM_DE,
        .pclk_gpio_num = PIN_NUM_PCLK,
        .data_gpio_nums = {
            PIN_NUM_DATA0,  // B3 (GPIO14)
            PIN_NUM_DATA1,  // B4 (GPIO38)
            PIN_NUM_DATA2,  // B5 (GPIO18)
            PIN_NUM_DATA3,  // B6 (GPIO17)
            PIN_NUM_DATA4,  // B7 (GPIO10)
            PIN_NUM_DATA5,  // G2 (GPIO39)
            PIN_NUM_DATA6,  // G3 (GPIO0)
            PIN_NUM_DATA7,  // G4 (GPIO45)
            PIN_NUM_DATA8,  // G5 (GPIO48)
            PIN_NUM_DATA9,  // G6 (GPIO47)
            PIN_NUM_DATA10, // G7 (GPIO21)
            PIN_NUM_DATA11, // R3 (GPIO1)
            PIN_NUM_DATA12, // R4 (GPIO2)
            PIN_NUM_DATA13, // R5 (GPIO42)
            PIN_NUM_DATA14, // R6 (GPIO41)
            PIN_NUM_DATA15, // R7 (GPIO40)
        },
        .disp_gpio_num = -1,
        .on_frame_trans_done = NULL,
        .user_ctx = NULL,
        .flags = {
            .fb_in_psram = 1, // Use PSRAM for frame buffer (we have 8MB PSRAM)
        },
    };

    if (Serial) {
        Serial.println("Creating RGB panel...");
        Serial.printf("Free heap before panel creation: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("Required frame buffer size: %d bytes\n", LCD_H_RES * LCD_V_RES * 2);
    }

    esp_err_t ret = esp_lcd_new_rgb_panel(&panel_config, &panel_handle);
    if (ret != ESP_OK) {
        if (Serial) {
            Serial.printf("ERROR: Failed to create RGB panel: %s (0x%x)\n", esp_err_to_name(ret), ret);
            Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("Frame buffer needed: %d bytes\n", LCD_H_RES * LCD_V_RES * 2);
            if (ret == ESP_ERR_NO_MEM) {
                Serial.println("Not enough memory for frame buffer!");
            }
        }
        return;
    }
    
    if (Serial) {
        Serial.println("RGB panel created successfully");
        Serial.printf("Free heap after panel creation: %d bytes\n", ESP.getFreeHeap());
    }

    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        if (Serial) {
            Serial.printf("ERROR: Failed to init RGB panel: %s\n", esp_err_to_name(ret));
        }
        return;
    }

    if (Serial) {
        Serial.println("RGB LCD panel initialized successfully!");
    }
}

void display_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    if (panel_handle != NULL) {
        // Calculate the dimensions
        int32_t x1 = area->x1;
        int32_t y1 = area->y1;
        int32_t x2 = area->x2;
        int32_t y2 = area->y2;
        
        // Draw the bitmap to the RGB LCD panel
        esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, color_p);
        
        // Debug output (only occasionally to avoid spam)
        static uint32_t flush_count = 0;
        flush_count++;
        if (flush_count % 500 == 0 && Serial) {
            Serial.printf("Display flush #%d: area (%d,%d) to (%d,%d) - %s\n", 
                          flush_count, x1, y1, x2, y2, 
                          (ret == ESP_OK) ? "OK" : esp_err_to_name(ret));
        }
    } else {
        // Debug output for when panel is not initialized
        static uint32_t flush_count = 0;
        flush_count++;
        if (flush_count % 1000 == 0 && Serial) {
            Serial.printf("Display flush #%d: panel not initialized\n", flush_count);
        }
    }
    
    // Tell LVGL that flushing is done
    lv_disp_flush_ready(disp_drv);
}

bool display_init(void) {
    if (Serial) {
        Serial.println("=== DISPLAY INITIALIZATION ===");
    }
    
    // Perform LCD reset sequence
    lcd_reset_sequence();
    
    // Initialize RGB LCD
    init_rgb_lcd();
    
    if (panel_handle == NULL) {
        if (Serial) {
            Serial.println("ERROR: Failed to initialize display");
        }
        return false;
    }
    
    // Initialize LVGL display buffer
    size_t buf_size = LCD_WIDTH * LCD_HEIGHT / 10;  // 1/10 screen buffer
    disp_draw_buf = (lv_color_t*)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!disp_draw_buf) {
        if (Serial) {
            Serial.println("ERROR: Failed to allocate display buffer");
        }
        return false;
    }
    
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, buf_size);
    
    // Initialize display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = display_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    
    if (Serial) {
        Serial.println("Display initialization complete");
    }
    
    return true;
}

esp_lcd_panel_handle_t* display_get_panel_handle(void) {
    return &panel_handle;
}

uint16_t display_get_width(void) {
    return LCD_WIDTH;
}

uint16_t display_get_height(void) {
    return LCD_HEIGHT;
}
