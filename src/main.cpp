#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>

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

// I2C pins for touch and CH422G - Corrected according to pinouts.txt
#define I2C_SDA     8   // TP_SDA (GPIO8)
#define I2C_SCL     9   // TP_SCL (GPIO9)
#define I2C_FREQ    400000

// Touch interrupt pin
#define TOUCH_INT   4   // TP_IRQ (GPIO4)

// CH422G I2C addresses
#define CH422G_CONFIG_ADDR  0x24
#define CH422G_DATA_ADDR    0x38

// LVGL display buffer
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

// RGB LCD panel handle
static esp_lcd_panel_handle_t panel_handle = NULL;

// RGB LCD timing configuration based on ESP32-S3-Touch-LCD-5 documentation
#define LCD_H_RES                   1024
#define LCD_V_RES                   600
// Critical anti-drift configuration from documentation
#define LCD_PIXEL_CLOCK_HZ          (16 * 1000 * 1000)  // 16MHz for stability
#define LCD_HSYNC_PULSE_WIDTH       30    // Enhanced from 10
#define LCD_HSYNC_BACK_PORCH        150   // Anti-drift value from docs
#define LCD_HSYNC_FRONT_PORCH       165   // Anti-drift value from docs  
#define LCD_VSYNC_PULSE_WIDTH       2     // Datasheet stable value
#define LCD_VSYNC_BACK_PORCH        23    // Datasheet stable value
#define LCD_VSYNC_FRONT_PORCH       12    // Datasheet stable value
// Bounce buffer to prevent display drift
#define LCD_RGB_BOUNCE_BUFFER_HEIGHT 10

// Function declarations
void setup_ch422g();
void lcd_reset_sequence();
void init_rgb_lcd();
void display_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void create_demo_ui();

void setup() {
    Serial.begin(115200);
    // Fix ESP32-S3 USB CDC timeout issues
    Serial.setTxTimeoutMs(0);  // Prevent delays when USB disconnected
    
    // Optional: Wait for serial connection with timeout (for debugging)
    unsigned long start = millis();
    while (!Serial && (millis() - start) < 2000) {
        delay(10);
    }
    
    if (Serial) {
        Serial.println("ESP32-S3-Touch-LCD-5 LVGL Demo Starting...");
    }

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    delay(100);

    // Setup CH422G I/O expander and LCD reset
    setup_ch422g();
    lcd_reset_sequence();

    // Initialize RGB LCD panel
    init_rgb_lcd();

    if (Serial) {
        Serial.println("Hardware setup complete, initializing LVGL...");
    }

    // Initialize LVGL
    lv_init();

    // Create display buffer
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * LCD_WIDTH * LCD_HEIGHT / 10, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!disp_draw_buf) {
        if (Serial) {
            Serial.println("ERROR: Failed to allocate display buffer!");
        }
        return;
    }
    
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);

    // Initialize display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = display_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 0;
    disp_drv.direct_mode = 0;
    
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    // Create demo UI
    create_demo_ui();

    if (Serial) {
        Serial.println("LVGL setup complete!");
    }
}

void loop() {
    lv_timer_handler();
    delay(5);
}

void setup_ch422g() {
    if (Serial) {
        Serial.println("Setting up CH422G I/O expander...");
    }
    
    // Configure CH422G for output mode
    Wire.beginTransmission(CH422G_CONFIG_ADDR);
    Wire.write(0x01);  // Set to output mode
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
        if (Serial) {
            Serial.println("CH422G config successful");
        }
    } else {
        if (Serial) {
            Serial.printf("CH422G config error: %d\n", error);
        }
    }
    delay(10);
    
    // Initial state: Enable display, LCD reset, and backlight (from documentation)
    Wire.beginTransmission(CH422G_DATA_ADDR);
    Wire.write(0x1E);  // LCD_BL=1, LCD_RST=1, DISP=1 (0x04 + 0x08 + 0x02 + base 0x10)
    error = Wire.endTransmission();
    if (error == 0) {
        if (Serial) {
            Serial.println("CH422G backlight and display enabled");
        }
    } else {
        if (Serial) {
            Serial.printf("CH422G enable error: %d\n", error);
        }
    }
    delay(10);
}

void lcd_reset_sequence() {
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

void init_rgb_lcd() {
    if (Serial) {
        Serial.println("Initializing RGB LCD panel...");
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
        .on_frame_trans_done = nullptr,
        .user_ctx = nullptr,
        .flags = {
            .fb_in_psram = 1, // Frame buffer in PSRAM
        },
    };

    esp_err_t ret = esp_lcd_new_rgb_panel(&panel_config, &panel_handle);
    if (ret != ESP_OK) {
        if (Serial) {
            Serial.printf("ERROR: Failed to create RGB panel: %s\n", esp_err_to_name(ret));
        }
        return;
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

void create_demo_ui() {
    // Set background color
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, LV_PART_MAIN);

    // Create main label
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "ESP32-S3-Touch-LCD-5\nLVGL + RGB Display\nDemo Running!");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -50);

    // Create test button
    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 200, 60);
    lv_obj_align_to(btn, label, LV_ALIGN_OUT_BOTTOM_MID, 0, 40);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x2196F3), LV_PART_MAIN);

    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Test Button");
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
    lv_obj_center(btn_label);

    // Add some colored rectangles to test display
    lv_obj_t *rect1 = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rect1, 100, 100);
    lv_obj_align(rect1, LV_ALIGN_TOP_LEFT, 20, 20);
    lv_obj_set_style_bg_color(rect1, lv_color_hex(0xFF5722), LV_PART_MAIN);
    lv_obj_set_style_border_width(rect1, 0, LV_PART_MAIN);

    lv_obj_t *rect2 = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rect2, 100, 100);
    lv_obj_align(rect2, LV_ALIGN_TOP_RIGHT, -20, 20);
    lv_obj_set_style_bg_color(rect2, lv_color_hex(0x4CAF50), LV_PART_MAIN);
    lv_obj_set_style_border_width(rect2, 0, LV_PART_MAIN);

    lv_obj_t *rect3 = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rect3, 100, 100);
    lv_obj_align(rect3, LV_ALIGN_BOTTOM_LEFT, 20, -20);
    lv_obj_set_style_bg_color(rect3, lv_color_hex(0xFFEB3B), LV_PART_MAIN);
    lv_obj_set_style_border_width(rect3, 0, LV_PART_MAIN);

    lv_obj_t *rect4 = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rect4, 100, 100);
    lv_obj_align(rect4, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    lv_obj_set_style_bg_color(rect4, lv_color_hex(0x9C27B0), LV_PART_MAIN);
    lv_obj_set_style_border_width(rect4, 0, LV_PART_MAIN);

    Serial.println("Demo UI created");
}
