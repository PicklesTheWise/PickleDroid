#include "main.h"

void setup_ch422g(void) {
    if (Serial) {
        Serial.println("Setting up CH422G I/O expander...");
    }
    
    // Configure CH422G for output mode
    Wire.beginTransmission(CH422G_CONFIG_ADDR);
    Wire.write(0x01); // Configure as outputs
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
        if (Serial) {
            Serial.println("CH422G configuration successful");
        }
        
        // Set initial state: LCD_RST high, BL high, TP_RST high
        Wire.beginTransmission(CH422G_DATA_ADDR);
        Wire.write(0x1E); // All outputs high
        Wire.endTransmission();
        
        if (Serial) {
            Serial.println("CH422G initial state set (all outputs high)");
        }
    } else {
        if (Serial) {
            Serial.printf("CH422G configuration failed with error: %d\n", error);
        }
    }
}

void gt911_reset_sequence(void) {
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

// Button event handler
static void btn_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    
    if (code == LV_EVENT_CLICKED) {
        Serial.println("=== BUTTON CLICKED! ===");
        Serial.printf("Button pressed at %lu ms\n", millis());
        
        // Change button color to indicate it was pressed
        static bool button_state = false;
        button_state = !button_state;
        
        if (button_state) {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF5722), LV_PART_MAIN); // Orange
            Serial.println("Button state: ACTIVE (Orange)");
        } else {
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x2196F3), LV_PART_MAIN); // Blue
            Serial.println("Button state: NORMAL (Blue)");
        }
        
        // Force display update
        lv_obj_invalidate(btn);
    }
}

void create_demo_ui(void) {
    // Clear any existing content first
    lv_obj_clean(lv_scr_act());
    
    // Set background color to dark blue
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, LV_PART_MAIN);

    // Create main title label
    lv_obj_t *label = lv_label_create(lv_scr_act());
    if (touch_get_calibration_active()) {
        lv_label_set_text(label, "ESP32-S3-Touch-LCD-5\nCalibrated Touch System\n\nTest the button below!");
    } else {
        lv_label_set_text(label, "ESP32-S3-Touch-LCD-5\nTouch System Ready\n\nType 'c' for calibration");
    }
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -80);

    // Create test button - centered on screen
    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 200, 60);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x2196F3), LV_PART_MAIN);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 20);
    
    // Add event handler to the button
    lv_obj_add_event_cb(btn, btn_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Test Touch");
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
    lv_obj_center(btn_label);
    
    // Add calibration status indicator
    lv_obj_t *status_label = lv_label_create(lv_scr_act());
    if (touch_get_calibration_active()) {
        lv_label_set_text(status_label, "✓ CALIBRATED");
        lv_obj_set_style_text_color(status_label, lv_color_hex(0x00FF00), 0);  // Green
    } else {
        lv_label_set_text(status_label, "⚠ NOT CALIBRATED");
        lv_obj_set_style_text_color(status_label, lv_color_hex(0xFF9800), 0);  // Orange
    }
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -20);

    // Force screen refresh
    lv_refr_now(NULL);
    
    Serial.println("Clean demo UI created - ready for calibration");
}

void process_serial_commands(void) {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();
        
        if (cmd == "debug") {
            bool debug_enabled = !touch_get_debug_enabled();
            touch_set_debug_enabled(debug_enabled);
            Serial.printf("Touch debug: %s\n", debug_enabled ? "ON" : "OFF");
        }
        else if (cmd == "c" || cmd == "cal" || cmd == "calibrate") {
            start_calibration();
        }
        else if (cmd == "r" || cmd == "restart") {
            Serial.println("Restarting system...");
            ESP.restart();
        }
        else if (cmd == "clear") {
            // Force clear screen and show normal UI
            create_demo_ui();
            Serial.println("Cleared screen and returned to normal UI");
        }
        else if (cmd == "test") {
            Serial.println("Testing GT911 direct I2C communication...");
            test_gt911_direct_i2c(0x14);
            test_gt911_direct_i2c(0x5D);
        }
        else if (cmd == "touch") {
            Serial.println("Starting GT911 direct touch test...");
            test_gt911_touch_direct();
        }
        else if (cmd == "regs") {
            Serial.println("Starting GT911 register analysis...");
            test_gt911_registers();
        }
        else if (cmd == "help") {
            Serial.println("\n=== Available Commands ===");
            Serial.println("debug - Toggle touch debug output");
            Serial.println("c/cal/calibrate - Start touch calibration");
            Serial.println("test - Test GT911 I2C communication");
            Serial.println("touch - Test GT911 touch reading directly");
            Serial.println("regs - Analyze GT911 registers");
            Serial.println("clear - Clear screen and return to normal UI");
            Serial.println("restart/r - Restart the system");
            Serial.println("help - Show this help message");
        }
        else if (cmd.length() > 0) {
            Serial.printf("Unknown command: '%s'. Type 'help' for available commands.\n", cmd.c_str());
        }
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

    // Initialize LVGL
    if (Serial) {
        Serial.println("Initializing LVGL...");
    }
    lv_init();
    if (Serial) {
        Serial.println("LVGL core initialized");
    }

    // Initialize display
    if (Serial) {
        Serial.println("Initializing display...");
    }
    if (!display_init()) {
        if (Serial) {
            Serial.println("ERROR: Failed to initialize display!");
        }
        return;
    }
    if (Serial) {
        Serial.println("Display initialized successfully");
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
    if (!touch_init()) {
        if (Serial) {
            Serial.println("ERROR: Failed to initialize touch!");
        }
        return;
    }
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
        Serial.println("=== SETUP COMPLETE ===");
        Serial.printf("Final free heap: %d bytes\n", ESP.getFreeHeap());
        Serial.println("System ready for operation");
        Serial.println("\n=== Touch Calibration System ===");
        Serial.println("Available commands:");
        Serial.println("  debug - Toggle debug output");
        Serial.println("  c/cal/calibrate - Start touch calibration");
        Serial.println("  test - Direct GT911 sensor test");
        Serial.println("  help - Show available commands");
        Serial.println("\nTo start calibration, type: c");
    }
}

void loop() {
    // Process serial commands for calibration
    process_serial_commands();
    
    // Handle touch interrupts
    handle_touch_interrupt();
    
    // Handle LVGL tasks - this is critical for touch input
    lv_timer_handler();
    
    // Small delay for stability
    delay(5);
}
