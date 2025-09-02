#include "touch_calibration.h"
#include <Wire.h>
#include <Arduino.h>

// Display resolution constants (from display.h)
#define DISPLAY_WIDTH  1024
#define DISPLAY_HEIGHT 600

// Global variables
static lv_indev_drv_t indev_drv;
static uint8_t gt911_i2c_addr = GT911_I2C_ADDR_DEFAULT;
static volatile bool touch_interrupt_flag = false;

// Interrupt-driven touch data
static volatile bool new_touch_data = false;
static volatile int16_t last_touch_x = 0;
static volatile int16_t last_touch_y = 0;
static volatile bool touch_pressed = false;

// Touch interrupt handler
void IRAM_ATTR touch_interrupt_handler(void) {
    touch_interrupt_flag = true;
}

bool test_gt911_direct_i2c(uint8_t addr) {
    Serial.printf("Testing direct I2C communication with GT911 at address 0x%02X...\n", addr);
    
    // First test: Simple address test
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    if (error != 0) {
        Serial.printf("Direct I2C test failed: error %d\n", error);
        return false;
    }
    
    // Second test: Try to read product ID (0x8140)
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
    
    Serial.printf("Failed to read GT911 product ID\n");
    return false;
}

void handle_touch_interrupt(void) {
    if (!touch_interrupt_flag) {
        return;  // No interrupt pending
    }
    
    // Clear interrupt flag immediately
    touch_interrupt_flag = false;
    
    // Read GT911 touch status register (0x814E)
    Wire.beginTransmission(gt911_i2c_addr);
    Wire.write(0x81);  // High byte
    Wire.write(0x4E);  // Low byte
    uint8_t error = Wire.endTransmission(false);
    
    if (error == 0) {
        Wire.requestFrom(gt911_i2c_addr, (uint8_t)1);
        if (Wire.available()) {
            uint8_t status = Wire.read();
            
            Serial.printf("INTERRUPT: GT911 status=0x%02X, buffer=%s, points=%d\n", 
                         status, (status & 0x80) ? "READY" : "EMPTY", status & 0x0F);
            
            // Check if touch is detected (buffer status & 0x80)
            if (status & 0x80) {
                uint8_t touch_points = status & 0x0F;
                
                if (touch_points > 0) {
                    // Read first touch point data (0x8150)
                    Wire.beginTransmission(gt911_i2c_addr);
                    Wire.write(0x81);  // High byte
                    Wire.write(0x50);  // Low byte
                    error = Wire.endTransmission(false);
                    
                    if (error == 0) {
                        Wire.requestFrom(gt911_i2c_addr, (uint8_t)8);
                        if (Wire.available() >= 8) {
                            uint8_t point_data[8];
                            for (int i = 0; i < 8; i++) {
                                point_data[i] = Wire.read();
                            }
                            
                            // Parse touch point data exactly like working implementation
                            uint16_t x = (point_data[1] << 8) | point_data[0];
                            uint16_t y = (point_data[3] << 8) | point_data[2];
                            uint16_t size = (point_data[5] << 8) | point_data[4];
                            
                            // Clamp coordinates to display resolution like working implementation
                            if (x > 1023) x = 1023;
                            if (y > 599) y = 599;
                            
                            Serial.printf("INTERRUPT: Touch detected - X=%d, Y=%d, ID=%d, Size=%d\n", 
                                         x, y, point_data[0], size);
                            Serial.printf("INTERRUPT: Raw bytes: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                                         point_data[0], point_data[1], point_data[2], point_data[3], 
                                         point_data[4], point_data[5], point_data[6], point_data[7]);
                            
                            // Store touch data for LVGL
                            last_touch_x = x;
                            last_touch_y = y;
                            touch_pressed = true;
                            new_touch_data = true;
                        }
                    }
                } else {
                    // No touch points - release
                    touch_pressed = false;
                    new_touch_data = true;
                    Serial.println("INTERRUPT: Touch released");
                }
                
                // Clear the touch buffer status by writing 0 to 0x814E
                Wire.beginTransmission(gt911_i2c_addr);
                Wire.write(0x81);  // High byte
                Wire.write(0x4E);  // Low byte
                Wire.write(0x00);  // Clear status
                Wire.endTransmission();
            } else {
                // Buffer empty but interrupt fired - could be release
                touch_pressed = false;
                new_touch_data = true;
                Serial.println("INTERRUPT: Buffer empty - touch released");
            }
        }
    } else {
        Serial.printf("INTERRUPT: I2C error %d\n", error);
    }
}

void touch_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data) {
    static uint32_t debug_counter = 0;
    static uint32_t last_print_time = 0;
    
    // Handle any pending touch interrupts
    handle_touch_interrupt();
    
    // Debug: Print every 1000 calls to verify function is being called
    debug_counter++;
    uint32_t now = millis();
    if (debug_counter % 1000 == 0 || (now - last_print_time) > 5000) {
        Serial.printf("touch_read called %lu times, interrupt_flag=%s, new_data=%s (time: %lu)\n", 
                     debug_counter, touch_interrupt_flag ? "SET" : "CLEAR", 
                     new_touch_data ? "YES" : "NO", now);
        last_print_time = now;
    }
    
    // Set LVGL data based on interrupt-driven touch state
    if (new_touch_data) {
        if (touch_pressed) {
            data->point.x = last_touch_x;
            data->point.y = last_touch_y;
            data->state = LV_INDEV_STATE_PR;
            Serial.printf("LVGL: Touch PRESSED at (%d,%d)\n", last_touch_x, last_touch_y);
        } else {
            data->point.x = last_touch_x;
            data->point.y = last_touch_y;
            data->state = LV_INDEV_STATE_REL;
            Serial.printf("LVGL: Touch RELEASED at (%d,%d)\n", last_touch_x, last_touch_y);
        }
        new_touch_data = false;  // Clear the flag
    } else {
        // No new touch data - maintain previous state
        data->point.x = last_touch_x;
        data->point.y = last_touch_y;
        data->state = touch_pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    }
}

bool touch_init(void) {
    Serial.println("Initializing GT911 touch controller...");
    
    // Configure touch controller interrupt pin
    pinMode(TOUCH_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TOUCH_INT), touch_interrupt_handler, FALLING);
    
    Serial.printf("GPIO4 (TOUCH_INT) configured as INPUT_PULLUP with FALLING edge interrupt\n");
    
    // Try GT911 addresses (0x14 first, then 0x5D)
    uint8_t gt911_addresses[] = {0x14, 0x5D};
    bool touch_initialized = false;
    
    for (int i = 0; i < 2 && !touch_initialized; i++) {
        Serial.printf("Trying GT911 I2C address: 0x%02X\n", gt911_addresses[i]);
        
        if (test_gt911_direct_i2c(gt911_addresses[i])) {
            Serial.printf("GT911 touch controller initialized successfully at address 0x%02X\n", gt911_addresses[i]);
            touch_initialized = true;
            gt911_i2c_addr = gt911_addresses[i];
        } else {
            Serial.printf("Failed to initialize GT911 at address 0x%02X\n", gt911_addresses[i]);
        }
    }
    
    if (!touch_initialized) {
        Serial.println("ERROR: Failed to initialize GT911 touch controller at any address");
        return false;
    }
    
    // Configure LVGL input device
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    lv_indev_drv_register(&indev_drv);
    
    Serial.println("Touch input device registered with LVGL");
    Serial.println("GT911 touch controller initialization complete!");
    
    return true;
}

// Simple stub functions for compatibility with main.cpp
bool touch_get_calibration_active(void) {
    return false;  // No calibration in simplified mode
}

bool touch_get_debug_enabled(void) {
    return true;   // Debug always enabled in simplified mode
}

void touch_set_debug_enabled(bool enabled) {
    Serial.printf("Touch debug %s (simplified mode - always enabled)\n", enabled ? "enabled" : "disabled");
}

void start_calibration(void) {
    Serial.println("Calibration not available in simplified GT911 mode");
    Serial.println("Using stock GT911 driver - no calibration needed");
}

void test_gt911_touch_direct(void) {
    Serial.println("=== GT911 Direct Touch Test ===");
    
    for (int i = 0; i < 20; i++) {  // Test for 10 seconds
        // Read touch status register (0x814E)
        Wire.beginTransmission(gt911_i2c_addr);
        Wire.write(0x81);  // High byte
        Wire.write(0x4E);  // Low byte
        uint8_t error = Wire.endTransmission(false);
        
        if (error == 0) {
            Wire.requestFrom(gt911_i2c_addr, (uint8_t)1);
            if (Wire.available()) {
                uint8_t status = Wire.read();
                Serial.printf("Test %d: Status=0x%02X, Buffer=%s, Points=%d, Interrupt=%s\n",
                             i + 1, status, 
                             (status & 0x80) ? "READY" : "EMPTY",
                             status & 0x0F,
                             touch_interrupt_flag ? "SET" : "CLEAR");
                
                if (status & 0x80) {
                    uint8_t touch_points = status & 0x0F;
                    
                    if (touch_points > 0) {
                        // Read touch point data (0x8150)
                        Wire.beginTransmission(gt911_i2c_addr);
                        Wire.write(0x81);
                        Wire.write(0x50);
                        error = Wire.endTransmission(false);
                        
                        if (error == 0) {
                            Wire.requestFrom(gt911_i2c_addr, (uint8_t)8);  // Read extra bytes for debugging
                            if (Wire.available() >= 6) {
                                uint8_t bytes[8];
                                int bytes_read = 0;
                                while (Wire.available() && bytes_read < 8) {
                                    bytes[bytes_read] = Wire.read();
                                    bytes_read++;
                                }
                                
                                if (bytes_read >= 6) {
                                    uint8_t track_id = bytes[0];
                                    uint8_t x_low = bytes[1];
                                    uint8_t x_high = bytes[2];
                                    uint8_t y_low = bytes[3];
                                    uint8_t y_high = bytes[4];
                                    uint8_t size = bytes[5];
                                    
                                    int16_t x = (x_high << 8) | x_low;
                                    int16_t y = (y_high << 8) | y_low;
                                    
                                    // Scale coordinates to display resolution
                                    int16_t scaled_x = map(x, 0, 32767, 0, DISPLAY_WIDTH - 1);
                                    int16_t scaled_y = map(y, 0, 32767, 0, DISPLAY_HEIGHT - 1);
                                    scaled_x = constrain(scaled_x, 0, DISPLAY_WIDTH - 1);
                                    scaled_y = constrain(scaled_y, 0, DISPLAY_HEIGHT - 1);
                                    
                                    Serial.printf("  -> Raw bytes (%d): ", bytes_read);
                                    for (int b = 0; b < bytes_read; b++) {
                                        Serial.printf("%02X ", bytes[b]);
                                    }
                                    Serial.println();
                                    Serial.printf("  -> Raw: ID=%d, X=%d (0x%04X), Y=%d (0x%04X), Size=%d\n", 
                                                 track_id, x, x, y, y, size);
                                    Serial.printf("  -> Scaled: X=%d, Y=%d (for %dx%d display)\n", 
                                                 scaled_x, scaled_y, DISPLAY_WIDTH, DISPLAY_HEIGHT);
                                }
                            }
                        }
                    }
                    
                    // Clear status
                    Wire.beginTransmission(gt911_i2c_addr);
                    Wire.write(0x81);
                    Wire.write(0x4E);
                    Wire.write(0x00);
                    Wire.endTransmission();
                }
                
                // Reset interrupt flag
                touch_interrupt_flag = false;
            }
        } else {
            Serial.printf("Test %d: I2C Error %d\n", i + 1, error);
        }
        
        delay(500);  // Wait 500ms between tests
    }
    
    Serial.println("=== Direct Touch Test Complete ===");
}

void test_gt911_registers(void) {
    Serial.println("=== GT911 Register Analysis ===");
    
    // Read multiple registers to understand the data layout
    uint16_t registers[] = {0x814E, 0x814F, 0x8150, 0x8151, 0x8152, 0x8153, 0x8154, 0x8155, 0x8156, 0x8157};
    int num_regs = sizeof(registers) / sizeof(registers[0]);
    
    for (int test = 0; test < 5; test++) {
        Serial.printf("\n--- Test %d ---\n", test + 1);
        
        for (int i = 0; i < num_regs; i++) {
            Wire.beginTransmission(gt911_i2c_addr);
            Wire.write((registers[i] >> 8) & 0xFF);  // High byte
            Wire.write(registers[i] & 0xFF);         // Low byte
            uint8_t error = Wire.endTransmission(false);
            
            if (error == 0) {
                Wire.requestFrom(gt911_i2c_addr, (uint8_t)1);
                if (Wire.available()) {
                    uint8_t value = Wire.read();
                    Serial.printf("Reg 0x%04X: 0x%02X (%3d)\n", registers[i], value, value);
                }
            } else {
                Serial.printf("Reg 0x%04X: I2C Error %d\n", registers[i], error);
            }
        }
        
        Serial.println("Now touch the screen...");
        delay(2000);  // Give time to touch
    }
    
    Serial.println("=== Register Analysis Complete ===");
}
