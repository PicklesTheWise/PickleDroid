#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
// #include <bb_captouch.h>  // Using direct GT911 implementation instead

// Touch calibration and filtering configuration
struct TouchCalibration {
    // GT911 raw coordinate ranges (based on observed data from ESP32-S3-Touch-LCD-5)
    uint16_t raw_x_min = 0;
    uint16_t raw_x_max = 65280;  // Observed max from test data
    uint16_t raw_y_min = 0;  
    uint16_t raw_y_max = 26369;  // Observed max from test data
    
    // Display resolution
    uint16_t display_width = 1024;
    uint16_t display_height = 600;
    
    // Touch filtering
    uint8_t filter_samples = 3;     // Number of samples to average
    uint16_t noise_threshold = 50;   // Minimum movement to register
    uint32_t debounce_time_ms = 50;  // Minimum time between touch events
    
    // Coordinate transformation flags
    bool swap_xy = false;
    bool mirror_x = false;
    bool mirror_y = false;
};

// Touch filtering state
struct TouchFilter {
    uint16_t x_history[5] = {0};
    uint16_t y_history[5] = {0};
    uint8_t history_index = 0;
    uint8_t history_count = 0;
    uint32_t last_touch_time = 0;
    bool touch_active = false;
    uint16_t last_filtered_x = 0;
    uint16_t last_filtered_y = 0;
};

// Global touch configuration instances
TouchCalibration touch_cal;
TouchFilter touch_filter;

// Debug mode for touch
bool touch_debug_enabled = false;

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

// Touch controller configuration  
// BBCapTouch touch;  // Disable bb_captouch - implement direct GT911 communication
uint8_t gt911_i2c_addr = 0x14;  // Current GT911 address

// Touch interrupt handling
volatile bool touch_interrupt_flag = false;
static uint32_t last_interrupt_time = 0;
static const uint32_t DEBOUNCE_MS = 10; // Debounce interrupt for 10ms

// LVGL display buffer
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

// LVGL touch input driver
static lv_indev_drv_t indev_drv;

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

// Direct GT911 I2C communication test
bool test_gt911_direct_i2c(uint8_t addr) {
    Serial.printf("Testing direct I2C communication with GT911 at address 0x%02X...\n", addr);
    
    // Test 1: Basic I2C communication
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.printf("Direct I2C test failed: error %d\n", error);
        return false;
    }
    
    // Test 2: Try to read Product ID (0x8140)
    Wire.beginTransmission(addr);
    Wire.write(0x81);  // High byte
    Wire.write(0x40);  // Low byte  
    error = Wire.endTransmission(false); // Keep connection alive
    
    if (error == 0) {
        Wire.requestFrom(addr, (uint8_t)4);
        if (Wire.available() >= 4) {
            char product_id[5] = {0};
            for (int i = 0; i < 4; i++) {
                product_id[i] = Wire.read();
            }
            Serial.printf("GT911 Product ID: %s\n", product_id);
            return true;
        }
    }
    
    Serial.printf("Failed to read GT911 Product ID, error: %d\n", error);
    return false;
}

// Direct GT911 touch reading test
bool read_gt911_touch_direct(uint8_t addr) {
    // Read touch status register (0x814E)
    Wire.beginTransmission(addr);
    Wire.write(0x81);  // High byte
    Wire.write(0x4E);  // Low byte
    uint8_t error = Wire.endTransmission(false);
    
    if (error != 0) {
        return false;
    }
    
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) {
        uint8_t status = Wire.read();
        uint8_t touch_count = status & 0x0F;
        
        if ((status & 0x80) && touch_count > 0) {
            // Touch detected, read coordinates from 0x814F
            Wire.beginTransmission(addr);
            Wire.write(0x81);  // High byte  
            Wire.write(0x4F);  // Low byte
            error = Wire.endTransmission(false);
            
            if (error == 0) {
                Wire.requestFrom(addr, (uint8_t)4);
                if (Wire.available() >= 4) {
                    uint16_t x = Wire.read() | (Wire.read() << 8);
                    uint16_t y = Wire.read() | (Wire.read() << 8);
                    
                    Serial.printf("DIRECT GT911 TOUCH: Count=%d, X=%d, Y=%d\n", touch_count, x, y);
                    
                    // Clear status register
                    Wire.beginTransmission(addr);
                    Wire.write(0x81);  // High byte
                    Wire.write(0x4E);  // Low byte
                    Wire.write(0x00);  // Clear status
                    Wire.endTransmission();
                    
                    return true;
                }
            }
        }
    }
    return false;
}

// GT911 Driver Implementation (following ESP-BSP implementation)
class GT911Driver {
private:
    uint8_t i2c_addr;
    bool initialized;
    
public:
    GT911Driver() : i2c_addr(0x14), initialized(false) {}
    
    bool init(uint8_t addr) {
        i2c_addr = addr;
        Serial.printf("GT911: Initializing at address 0x%02X\n", addr);
        
        // Test basic I2C communication
        Wire.beginTransmission(i2c_addr);
        uint8_t error = Wire.endTransmission();
        if (error != 0) {
            Serial.printf("GT911: I2C communication failed at address 0x%02X, error: %d\n", addr, error);
            return false;
        }
        
        // Read Product ID (registers 0x8140-0x8142)
        uint8_t product_id[3];
        if (!readRegister(0x8140, product_id, 3)) {
            Serial.printf("GT911: Failed to read Product ID at address 0x%02X\n", addr);
            return false;
        }
        
        Serial.printf("GT911: Product ID: %c%c%c (0x%02X%02X%02X)\n", 
                     product_id[0], product_id[1], product_id[2],
                     product_id[0], product_id[1], product_id[2]);
        
        // Read current config version (register 0x8047)
        uint8_t config_version;
        if (readRegister(0x8047, &config_version, 1)) {
            Serial.printf("GT911: Current config version: 0x%02X\n", config_version);
        }
        
        initialized = true;
        Serial.println("GT911: Initialization completed successfully");
        return true;
    }
    
    typedef enum {
        SAMPLE_FAILED = 0,
        SAMPLE_SUCCESS = 1
    } sample_result_t;
    
    sample_result_t getSamples(uint16_t *x, uint16_t *y, uint8_t *points) {
        if (!initialized) return SAMPLE_FAILED;
        
        *points = 0;
        
        // Read status register (0x814E)
        uint8_t status;
        if (!readRegister(0x814E, &status, 1)) {
            return SAMPLE_FAILED;
        }
        
        // Check if there's valid touch data available
        if ((status & 0x80) == 0) {
            // No touch data ready
            return SAMPLE_FAILED;
        }
        
        // Get number of touch points
        uint8_t touch_count = status & 0x0F;
        if (touch_count == 0 || touch_count > 5) {
            // Invalid touch count, clear status
            clearStatusRegister();
            return SAMPLE_FAILED;
        }
        
        // Read touch data for first point (starting at 0x814F)
        uint8_t touch_data[8];
        if (!readRegister(0x814F, touch_data, 8)) {
            clearStatusRegister();
            return SAMPLE_FAILED;
        }
        
        // Extract coordinates (GT911 format: LSB first)
        *x = ((uint16_t)touch_data[1] << 8) | touch_data[0];
        *y = ((uint16_t)touch_data[3] << 8) | touch_data[2];
        *points = 1;
        
        // Clear status register to acknowledge the touch data
        clearStatusRegister();
        
        return SAMPLE_SUCCESS;
    }

private:
    bool readRegister(uint16_t reg, uint8_t* data, uint8_t length) {
        Wire.beginTransmission(i2c_addr);
        Wire.write((reg >> 8) & 0xFF);
        Wire.write(reg & 0xFF);
        uint8_t error = Wire.endTransmission(false);
        
        if (error == 0) {
            Wire.requestFrom(i2c_addr, length);
            if (Wire.available() >= length) {
                for (int i = 0; i < length; i++) {
                    data[i] = Wire.read();
                }
                return true;
            }
        }
        return false;
    }
    
    bool writeRegister(uint16_t reg, uint8_t data) {
        Wire.beginTransmission(i2c_addr);
        Wire.write((reg >> 8) & 0xFF);
        Wire.write(reg & 0xFF);
        Wire.write(data);
        return Wire.endTransmission() == 0;
    }
    
    void clearStatusRegister() {
        // Clear the status register to acknowledge touch data read
        writeRegister(0x814E, 0x00);
    }
};

GT911Driver gt911;
void setup_ch422g();
void lcd_reset_sequence();
void gt911_reset_sequence();
void init_rgb_lcd();
void display_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void init_touch();
void touch_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
void create_demo_ui();

// Manual calibration functions
void print_touch_calibration() {
    Serial.println("=== Current Touch Calibration ===");
    Serial.printf("Raw X range: %d - %d\n", touch_cal.raw_x_min, touch_cal.raw_x_max);
    Serial.printf("Raw Y range: %d - %d\n", touch_cal.raw_y_min, touch_cal.raw_y_max);
    Serial.printf("Display: %dx%d\n", touch_cal.display_width, touch_cal.display_height);
    Serial.printf("Transformations: swap_xy=%s, mirror_x=%s, mirror_y=%s\n",
                 touch_cal.swap_xy ? "YES" : "NO",
                 touch_cal.mirror_x ? "YES" : "NO", 
                 touch_cal.mirror_y ? "YES" : "NO");
    Serial.printf("Filter: %d samples, noise threshold: %d, debounce: %dms\n",
                 touch_cal.filter_samples, touch_cal.noise_threshold, touch_cal.debounce_time_ms);
    Serial.println("==================================");
}

void reset_touch_calibration() {
    touch_cal.raw_x_min = 0;
    touch_cal.raw_x_max = 32767;
    touch_cal.raw_y_min = 0;
    touch_cal.raw_y_max = 32767;
    touch_cal.swap_xy = false;
    touch_cal.mirror_x = false;
    touch_cal.mirror_y = false;
    Serial.println("Touch calibration reset to defaults");
}

// Serial command processor for calibration
void process_serial_commands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();
        
        if (cmd == "cal") {
            print_touch_calibration();
        }
        else if (cmd == "reset") {
            reset_touch_calibration();
        }
        else if (cmd == "debug") {
            touch_debug_enabled = !touch_debug_enabled;
            Serial.printf("Touch debug: %s\n", touch_debug_enabled ? "ON" : "OFF");
        }
        else if (cmd == "test") {
            // Direct GT911 test
            Serial.println("=== Direct GT911 Test ===");
            for (int i = 0; i < 10; i++) {
                uint16_t x, y;
                uint8_t points;
                GT911Driver::sample_result_t result = gt911.getSamples(&x, &y, &points);
                Serial.printf("Test %d: result=%s, points=%d", 
                             i+1, 
                             (result == GT911Driver::SAMPLE_SUCCESS) ? "SUCCESS" : "FAILED",
                             points);
                if (result == GT911Driver::SAMPLE_SUCCESS && points > 0) {
                    Serial.printf(", coordinates=(%d,%d)", x, y);
                }
                Serial.println();
                delay(100);
            }
            Serial.println("=== Test Complete ===");
        }
        else if (cmd.startsWith("bounds ")) {
            // Format: bounds x_min x_max y_min y_max
            sscanf(cmd.c_str(), "bounds %hu %hu %hu %hu", 
                   &touch_cal.raw_x_min, &touch_cal.raw_x_max,
                   &touch_cal.raw_y_min, &touch_cal.raw_y_max);
            print_touch_calibration();
        }
        else if (cmd == "swap") {
            touch_cal.swap_xy = !touch_cal.swap_xy;
            Serial.printf("Swap XY: %s\n", touch_cal.swap_xy ? "ON" : "OFF");
        }
        else if (cmd == "mirrorx") {
            touch_cal.mirror_x = !touch_cal.mirror_x;
            Serial.printf("Mirror X: %s\n", touch_cal.mirror_x ? "ON" : "OFF");
        }
        else if (cmd == "mirrory") {
            touch_cal.mirror_y = !touch_cal.mirror_y;
            Serial.printf("Mirror Y: %s\n", touch_cal.mirror_y ? "ON" : "OFF");
        }
        else if (cmd == "help") {
            Serial.println("Touch Calibration Commands:");
            Serial.println("  cal - Show current calibration");
            Serial.println("  reset - Reset to defaults");
            Serial.println("  debug - Toggle debug output");
            Serial.println("  test - Direct GT911 sensor test");
            Serial.println("  bounds x_min x_max y_min y_max - Set coordinate bounds");
            Serial.println("  swap - Toggle XY coordinate swap");
            Serial.println("  mirrorx - Toggle X mirroring");
            Serial.println("  mirrory - Toggle Y mirroring");
            Serial.println("  help - Show this help");
        }
    }
}

// Touch interrupt service routine (ISR)
void IRAM_ATTR touch_interrupt_handler() {
    uint32_t current_time = millis();
    if (current_time - last_interrupt_time > DEBOUNCE_MS) {
        touch_interrupt_flag = true;
        last_interrupt_time = current_time;
    }
}

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
        Serial.println("=== ESP32-S3-Touch-LCD-5 LVGL Demo Starting ===");
        Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
        Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
    }

    // Initialize I2C with pull-ups enabled (as per documentation)
    if (Serial) {
        Serial.printf("Initializing I2C on SDA=%d, SCL=%d, Freq=%d (with pull-ups)...\n", I2C_SDA, I2C_SCL, I2C_FREQ);
    }
    
    // Configure I2C pins with pull-ups before Wire.begin()
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    delay(100);
    if (Serial) {
        Serial.println("I2C initialized successfully");
    }

    // Setup CH422G I/O expander and LCD reset
    if (Serial) {
        Serial.println("Setting up CH422G I/O expander...");
    }
    setup_ch422g();
    
    if (Serial) {
        Serial.println("Performing LCD reset sequence...");
    }
    lcd_reset_sequence();

    // Initialize RGB LCD panel
    if (Serial) {
        Serial.println("Initializing RGB LCD panel...");
        Serial.printf("Free heap before LCD init: %d bytes\n", ESP.getFreeHeap());
    }
    init_rgb_lcd();

    if (Serial) {
        Serial.println("Hardware setup complete, initializing LVGL...");
        Serial.printf("Free heap after LCD init: %d bytes\n", ESP.getFreeHeap());
    }

    // Initialize LVGL
    if (Serial) {
        Serial.println("Initializing LVGL...");
    }
    lv_init();
    if (Serial) {
        Serial.println("LVGL core initialized");
    }

    // Create display buffer
    if (Serial) {
        Serial.printf("Allocating display buffer: %d bytes\n", sizeof(lv_color_t) * LCD_WIDTH * LCD_HEIGHT / 10);
        Serial.printf("Free heap before buffer allocation: %d bytes\n", ESP.getFreeHeap());
    }
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * LCD_WIDTH * LCD_HEIGHT / 10, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!disp_draw_buf) {
        if (Serial) {
            Serial.println("ERROR: Failed to allocate display buffer!");
            Serial.printf("Requested: %d bytes, Available: %d bytes\n", 
                         sizeof(lv_color_t) * LCD_WIDTH * LCD_HEIGHT / 10, ESP.getFreeHeap());
        }
        return;
    }
    if (Serial) {
        Serial.println("Display buffer allocated successfully");
        Serial.printf("Free heap after buffer allocation: %d bytes\n", ESP.getFreeHeap());
    }
    
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);
    if (Serial) {
        Serial.println("LVGL display buffer initialized");
    }

    // Initialize display driver
    if (Serial) {
        Serial.println("Initializing LVGL display driver...");
    }
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = display_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 0;
    disp_drv.direct_mode = 0;
    
    if (Serial) {
        Serial.println("Registering LVGL display driver...");
    }
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    if (Serial) {
        Serial.printf("Display driver registered: %p\n", disp);
    }

    // Initialize touch input
    if (Serial) {
        Serial.println("Initializing touch input...");
        Serial.println("Performing GT911 reset sequence...");
    }
    gt911_reset_sequence();
    
    if (Serial) {
        Serial.println("Initializing GT911 touch controller...");
    }
    init_touch();
    if (Serial) {
        Serial.println("Touch input initialized");
    }

    // Create demo UI
    if (Serial) {
        Serial.println("Creating demo UI...");
    }
    create_demo_ui();
    if (Serial) {
        Serial.println("Demo UI created successfully");
    }

    if (Serial) {
        Serial.println("=== LVGL SETUP COMPLETE ===");
        Serial.printf("Final free heap: %d bytes\n", ESP.getFreeHeap());
        Serial.println("System ready for operation");
        Serial.println("\n=== ENHANCED TOUCH CALIBRATION SYSTEM ===");
        Serial.println("Type 'help' for calibration commands");
        print_touch_calibration();
    }
}

void loop() {
    // Process serial commands for calibration
    process_serial_commands();
    
    // Handle LVGL tasks - this is critical for touch input
    lv_timer_handler();
    
    // Check for touch interrupt (much more efficient than polling)
    if (touch_interrupt_flag) {
        touch_interrupt_flag = false; // Clear the flag
        
        // Process touch when interrupt occurs
        lv_indev_data_t dummy_data;
        touch_read(nullptr, &dummy_data);
        
        if (touch_debug_enabled) {
            Serial.println("Touch interrupt triggered - processing touch data");
        }
    }
    
    // Occasional polling as backup (every 100ms) in case interrupt is missed
    static uint32_t last_backup_poll = 0;
    if (millis() - last_backup_poll > 100) {
        lv_indev_data_t dummy_data;
        touch_read(nullptr, &dummy_data);
        last_backup_poll = millis();
    }
    
    // Force periodic touch check for debugging (less frequent)
    static uint32_t last_force_check = 0;
    if (touch_debug_enabled && (millis() - last_force_check > 2000)) {
        Serial.println("Debug: Periodic touch status check...");
        last_force_check = millis();
    }
    
    // Small delay for stability
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

// GT911 touch controller reset sequence via CH422G
void gt911_reset_sequence() {
    if (Serial) {
        Serial.println("Performing GT911 reset sequence for address 0x14...");
    }
    
    // Configure INT pin as OUTPUT temporarily for address selection
    pinMode(TOUCH_INT, OUTPUT);
    digitalWrite(TOUCH_INT, LOW);  // LOW for address 0x14 (0x28/0x29)
    
    // Pull TP_RST low (reset active) via CH422G IO1
    Wire.beginTransmission(CH422G_DATA_ADDR);
    Wire.write(0x2C);  // 0x2E & ~0x02 (clear TP_RST bit)
    Wire.endTransmission();
    delay(100);  // 100ms reset hold with INT LOW
    
    // Pull TP_RST high (reset inactive) while keeping INT LOW
    Wire.beginTransmission(CH422G_DATA_ADDR);
    Wire.write(0x2E);  // 0x2E | 0x02 (set TP_RST bit)
    Wire.endTransmission();
    delay(50);   // 50ms with INT LOW to set address
    
    // Now reconfigure INT pin as INPUT_PULLUP for normal operation
    pinMode(TOUCH_INT, INPUT_PULLUP);
    delay(150);  // Additional stabilization
    
    if (Serial) {
        Serial.println("GT911 reset sequence complete - address 0x14 selected");
    }
}

void init_rgb_lcd() {
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
        .on_frame_trans_done = nullptr,
        .user_ctx = nullptr,
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

// Initialize touch controller
void init_touch() {
    if (Serial) {
        Serial.println("Initializing GT911 touch controller...");
    }
    
    // Configure touch controller interrupt pin as INPUT with PULLUP (standard for GT911)
    pinMode(TOUCH_INT, INPUT_PULLUP);
    
    // Attach interrupt for touch detection (GT911 interrupt is active LOW)
    attachInterrupt(digitalPinToInterrupt(TOUCH_INT), touch_interrupt_handler, FALLING);
    
    if (Serial) {
        Serial.printf("GPIO4 (TOUCH_INT) configured as INPUT_PULLUP with FALLING edge interrupt\n");
    }
    
    // Try GT911 addresses (0x14 first, then 0x5D)
    uint8_t gt911_addresses[] = {0x14, 0x5D};
    bool touch_initialized = false;
    
    for (int i = 0; i < 2 && !touch_initialized; i++) {
        if (Serial) {
            Serial.printf("Trying GT911 I2C address: 0x%02X\n", gt911_addresses[i]);
        }
        
        if (gt911.init(gt911_addresses[i])) {
            if (Serial) {
                Serial.printf("GT911 touch controller initialized successfully at address 0x%02X\n", gt911_addresses[i]);
                
                // Test GT911 getSamples() functionality
                Serial.printf("Testing GT911 getSamples()...\n");
                uint16_t x = 0, y = 0;
                uint8_t points = 0;
                GT911Driver::sample_result_t test_result = gt911.getSamples(&x, &y, &points);
                Serial.printf("Initial getSamples() test: %s", (test_result == GT911Driver::SAMPLE_SUCCESS) ? "SUCCESS" : "FAILED");
                if (test_result == GT911Driver::SAMPLE_SUCCESS) {
                    Serial.printf(", Points: %d", points);
                    if (points > 0) {
                        Serial.printf(", X: %d, Y: %d", x, y);
                    }
                }
                Serial.println();
            }
            touch_initialized = true;
            gt911_i2c_addr = gt911_addresses[i];
        } else {
            if (Serial) {
                Serial.printf("Failed to initialize GT911 at address 0x%02X\n", gt911_addresses[i]);
            }
        }
    }
    
    if (!touch_initialized) {
        if (Serial) {
            Serial.println("ERROR: Failed to initialize GT911 touch controller at any address");
        }
        return;
    }
    
    // Initialize LVGL input device driver
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    lv_indev_t *indev = lv_indev_drv_register(&indev_drv);
    
    if (Serial) {
        Serial.printf("LVGL input device registered: %p\n", indev);
        Serial.println("Touch input driver registered with LVGL");
        
        // Test the touch_read function manually
        Serial.println("Testing touch_read function manually...");
        lv_indev_data_t test_data;
        touch_read(&indev_drv, &test_data);
        Serial.printf("Manual test - touch_read completed, state: %s\n", 
                     (test_data.state == LV_INDEV_STATE_PR) ? "PRESSED" : "RELEASED");
    }
}

// Calibrate raw coordinates to display coordinates
void calibrate_coordinates(uint16_t raw_x, uint16_t raw_y, uint16_t *cal_x, uint16_t *cal_y) {
    // Apply coordinate transformations first
    uint16_t work_x = raw_x;
    uint16_t work_y = raw_y;
    
    // For GT911 on ESP32-S3-Touch-LCD-5, coordinates may need inversion
    // Try swapping X and Y coordinates
    if (touch_cal.swap_xy) {
        uint16_t temp = work_x;
        work_x = work_y;
        work_y = temp;
    }
    
    // Mirror coordinates if needed
    if (touch_cal.mirror_x) {
        work_x = touch_cal.raw_x_max - work_x + touch_cal.raw_x_min;
    }
    
    if (touch_cal.mirror_y) {
        work_y = touch_cal.raw_y_max - work_y + touch_cal.raw_y_min;
    }
    
    // Scale to display coordinates with better mapping
    if (touch_cal.raw_x_max > touch_cal.raw_x_min) {
        *cal_x = map(work_x, touch_cal.raw_x_min, touch_cal.raw_x_max, 0, touch_cal.display_width - 1);
    } else {
        *cal_x = 0;
    }
    
    if (touch_cal.raw_y_max > touch_cal.raw_y_min) {
        *cal_y = map(work_y, touch_cal.raw_y_min, touch_cal.raw_y_max, 0, touch_cal.display_height - 1);
    } else {
        *cal_y = 0;
    }
    
    // Clamp to display bounds
    *cal_x = constrain(*cal_x, 0, touch_cal.display_width - 1);
    *cal_y = constrain(*cal_y, 0, touch_cal.display_height - 1);
}

// Apply filtering to touch coordinates
bool filter_touch_coordinates(uint16_t raw_x, uint16_t raw_y, uint16_t *filtered_x, uint16_t *filtered_y) {
    uint32_t current_time = millis();
    
    // Debounce check
    if (touch_filter.touch_active && (current_time - touch_filter.last_touch_time) < touch_cal.debounce_time_ms) {
        return false;
    }
    
    // Add to history
    touch_filter.x_history[touch_filter.history_index] = raw_x;
    touch_filter.y_history[touch_filter.history_index] = raw_y;
    touch_filter.history_index = (touch_filter.history_index + 1) % touch_cal.filter_samples;
    
    if (touch_filter.history_count < touch_cal.filter_samples) {
        touch_filter.history_count++;
    }
    
    // Calculate average
    uint32_t sum_x = 0, sum_y = 0;
    uint8_t samples_to_use = min(touch_filter.history_count, touch_cal.filter_samples);
    
    for (uint8_t i = 0; i < samples_to_use; i++) {
        sum_x += touch_filter.x_history[i];
        sum_y += touch_filter.y_history[i];
    }
    
    uint16_t avg_x = sum_x / samples_to_use;
    uint16_t avg_y = sum_y / samples_to_use;
    
    // Noise filtering - check if movement is significant
    if (touch_filter.touch_active) {
        uint16_t dx = abs((int16_t)avg_x - (int16_t)touch_filter.last_filtered_x);
        uint16_t dy = abs((int16_t)avg_y - (int16_t)touch_filter.last_filtered_y);
        
        if (dx < touch_cal.noise_threshold && dy < touch_cal.noise_threshold) {
            // Movement too small, use last coordinates
            *filtered_x = touch_filter.last_filtered_x;
            *filtered_y = touch_filter.last_filtered_y;
            return true;
        }
    }
    
    // Update state
    touch_filter.last_filtered_x = avg_x;
    touch_filter.last_filtered_y = avg_y;
    touch_filter.last_touch_time = current_time;
    touch_filter.touch_active = true;
    
    *filtered_x = avg_x;
    *filtered_y = avg_y;
    
    return true;
}

// Reset touch filter when touch is released
void reset_touch_filter() {
    touch_filter.touch_active = false;
    touch_filter.history_count = 0;
    touch_filter.history_index = 0;
}

// Auto-calibration based on observed touch data
void update_calibration_bounds(uint16_t raw_x, uint16_t raw_y) {
    static bool calibration_active = true;
    static uint32_t calibration_start_time = 0;
    static const uint32_t CALIBRATION_DURATION_MS = 30000; // 30 seconds
    static bool initial_message_shown = false;
    
    if (calibration_start_time == 0) {
        calibration_start_time = millis();
    }
    
    if (!initial_message_shown && calibration_active) {
        Serial.println("Touch calibration started - touch all corners of the screen");
        initial_message_shown = true;
    }
    
    if (calibration_active && (millis() - calibration_start_time) < CALIBRATION_DURATION_MS) {
        // Update bounds based on observed data
        bool bounds_updated = false;
        
        if (raw_x < touch_cal.raw_x_min) {
            touch_cal.raw_x_min = raw_x;
            bounds_updated = true;
        }
        if (raw_x > touch_cal.raw_x_max) {
            touch_cal.raw_x_max = raw_x;
            bounds_updated = true;
        }
        if (raw_y < touch_cal.raw_y_min) {
            touch_cal.raw_y_min = raw_y;
            bounds_updated = true;
        }
        if (raw_y > touch_cal.raw_y_max) {
            touch_cal.raw_y_max = raw_y;
            bounds_updated = true;
        }
    } else if (calibration_active) {
        calibration_active = false;
        Serial.printf("Calibration complete - X range: %d-%d, Y range: %d-%d\n",
                     touch_cal.raw_x_min, touch_cal.raw_x_max,
                     touch_cal.raw_y_min, touch_cal.raw_y_max);
    }
}

// Touch read callback for LVGL (improved version)
void touch_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    static uint32_t last_debug_time = 0;
    static uint32_t call_count = 0;
    static bool first_call = true;
    
    call_count++;
    
    // Show that function is being called on first call
    if (first_call) {
        Serial.println("touch_read() function is being called by LVGL");
        first_call = false;
    }
    
    // Read touch data from GT911
    uint16_t raw_x = 0, raw_y = 0;
    uint8_t points = 0;
    GT911Driver::sample_result_t result = gt911.getSamples(&raw_x, &raw_y, &points);
    
    // Set default state
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_REL;
    
    // Always show debug when enabled and either touching or every 1000 calls (instead of 100)
    bool should_debug = touch_debug_enabled && 
                       ((call_count % 1000 == 0) || 
                        (result == GT911Driver::SAMPLE_SUCCESS && points > 0));
    
    if (should_debug) {
        Serial.printf("touch_read[%d]: result=%s, points=%d", 
                     call_count, 
                     (result == GT911Driver::SAMPLE_SUCCESS) ? "SUCCESS" : "FAILED",
                     points);
        if (result == GT911Driver::SAMPLE_SUCCESS && points > 0) {
            Serial.printf(", raw=(%d,%d)", raw_x, raw_y);
        }
        Serial.println();
        last_debug_time = millis();
    }
    
    if (result == GT911Driver::SAMPLE_SUCCESS && points > 0) {
        // Update calibration bounds during initial period
        update_calibration_bounds(raw_x, raw_y);
        
        // Apply filtering
        uint16_t filtered_x, filtered_y;
        if (filter_touch_coordinates(raw_x, raw_y, &filtered_x, &filtered_y)) {
            // Apply calibration
            uint16_t cal_x, cal_y;
            calibrate_coordinates(filtered_x, filtered_y, &cal_x, &cal_y);
            
            // Update LVGL data
            data->point.x = cal_x;
            data->point.y = cal_y;
            data->state = LV_INDEV_STATE_PR;
            
            last_x = cal_x;
            last_y = cal_y;
            
            // Always show successful touches
            Serial.printf("TOUCH DETECTED: Raw(%d,%d) -> Filtered(%d,%d) -> Display(%d,%d)\n",
                         raw_x, raw_y, filtered_x, filtered_y, cal_x, cal_y);
        }
    } else {
        // No touch detected - reset filter
        reset_touch_filter();
    }
}

void create_demo_ui() {
    // Set background color
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, LV_PART_MAIN);

    // Create main label
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "ESP32-S3-Touch-LCD-5\nLVGL + RGB Display\nDemo Running!");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
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
