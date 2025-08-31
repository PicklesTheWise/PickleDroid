/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "waveshare_rgb_lcd_port.h"
#include "rom/ets_sys.h"  // For ets_delay_us() function

// VSYNC event callback function
IRAM_ATTR static bool rgb_lcd_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    return lvgl_port_notify_rgb_vsync();
}

/**
 * @brief I2C master initialization - used for CH422G and touch controller
 * Now checks if I2C is already initialized to avoid driver conflicts
 */
esp_err_t i2c_master_init(void)
{
    // Check if I2C driver is already installed by testing the driver install
    // If it fails with ESP_ERR_INVALID_STATE, the driver is already installed
    esp_err_t test_result = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    
    if (test_result == ESP_ERR_INVALID_STATE) {
        // I2C driver already installed
        ESP_LOGI(RGB_PORT_TAG, "I2C driver already installed and working");
        return ESP_OK;
    } else if (test_result == ESP_OK) {
        // Driver was just installed, need to configure parameters
        ESP_LOGI(RGB_PORT_TAG, "I2C driver installed successfully");
        
        i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };
        
        // Configure I2C parameters
        return i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    } else {
        // Some other error occurred
        ESP_LOGE(RGB_PORT_TAG, "I2C driver install failed with error: %s", esp_err_to_name(test_result));
        return test_result;
    }
}

#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911
// GPIO initialization
void gpio_init(void)
{
    // Zero-initialize the config structure
    gpio_config_t io_conf = {};
    // Disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // Set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    gpio_config(&io_conf);
}

// Reset the touch screen
void waveshare_esp32_s3_touch_reset()
{
    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // Reset the touch screen. It is recommended to reset the touch screen before using it.
    write_buf = 0x2C;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    ets_delay_us(100 * 1000);
    gpio_set_level(GPIO_INPUT_IO_4, 0);
    ets_delay_us(100 * 1000);
    write_buf = 0x2E;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    ets_delay_us(200 * 1000);
}

#endif

// Initialize RGB LCD
esp_err_t waveshare_esp32_s3_rgb_lcd_init()
{
    ESP_LOGI(RGB_PORT_TAG, "Install RGB LCD panel driver"); // Log the start of the RGB LCD panel driver installation
    esp_lcd_panel_handle_t panel_handle = NULL;    // Declare a handle for the LCD panel
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT, // Set the clock source for the panel
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ, // Pixel clock frequency
            .h_res = EXAMPLE_LCD_H_RES,            // Horizontal resolution
            .v_res = EXAMPLE_LCD_V_RES,            // Vertical resolution
#if ESP_PANEL_USE_1024_600_LCD
            .hsync_back_porch = 150, // Horizontal back porch (moderate compensation)
            .hsync_front_porch = 165, // Horizontal front porch (moderate compensation)
            .hsync_pulse_width = 30, // Horizontal pulse width
            .vsync_back_porch = 23,  // Vertical back porch
            .vsync_front_porch = 12,  // Vertical front porch
            .vsync_pulse_width = 2,  // Vertical pulse width
#else
            .hsync_pulse_width = 4, // Horizontal sync pulse width
            .hsync_back_porch = 8,  // Horizontal back porch
            .hsync_front_porch = 8, // Horizontal front porch
            .vsync_pulse_width = 4, // Vertical sync pulse width
            .vsync_back_porch = 8,  // Vertical back porch
            .vsync_front_porch = 8, // Vertical front porch
#endif
            .flags = {
                .pclk_active_neg = 1, // Active low pixel clock
            },
        },
        .data_width = EXAMPLE_RGB_DATA_WIDTH,                    // Data width for RGB
        .bits_per_pixel = EXAMPLE_RGB_BIT_PER_PIXEL,             // Bits per pixel
        .num_fbs = LVGL_PORT_LCD_RGB_BUFFER_NUMS,                // Number of frame buffers
        .bounce_buffer_size_px = EXAMPLE_RGB_BOUNCE_BUFFER_SIZE, // Bounce buffer size in pixels
        .sram_trans_align = 4,                                   // SRAM transaction alignment
        .psram_trans_align = 64,                                 // PSRAM transaction alignment
        .hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC,              // GPIO number for horizontal sync
        .vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC,              // GPIO number for vertical sync
        .de_gpio_num = EXAMPLE_LCD_IO_RGB_DE,                    // GPIO number for data enable
        .pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK,                // GPIO number for pixel clock
        .disp_gpio_num = EXAMPLE_LCD_IO_RGB_DISP,                // GPIO number for display
        .data_gpio_nums = {
            EXAMPLE_LCD_IO_RGB_DATA0,
            EXAMPLE_LCD_IO_RGB_DATA1,
            EXAMPLE_LCD_IO_RGB_DATA2,
            EXAMPLE_LCD_IO_RGB_DATA3,
            EXAMPLE_LCD_IO_RGB_DATA4,
            EXAMPLE_LCD_IO_RGB_DATA5,
            EXAMPLE_LCD_IO_RGB_DATA6,
            EXAMPLE_LCD_IO_RGB_DATA7,
            EXAMPLE_LCD_IO_RGB_DATA8,
            EXAMPLE_LCD_IO_RGB_DATA9,
            EXAMPLE_LCD_IO_RGB_DATA10,
            EXAMPLE_LCD_IO_RGB_DATA11,
            EXAMPLE_LCD_IO_RGB_DATA12,
            EXAMPLE_LCD_IO_RGB_DATA13,
            EXAMPLE_LCD_IO_RGB_DATA14,
            EXAMPLE_LCD_IO_RGB_DATA15,
        },
        .flags = {
            .fb_in_psram = 1, // Use PSRAM for framebuffer
        },
    };

    // Create a new RGB panel with the specified configuration
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(RGB_PORT_TAG, "Initialize RGB LCD panel");         // Log the initialization of the RGB LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle)); // Initialize the LCD panel
    
    // Add stabilization delay to allow panel timing to settle
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay for panel stabilization

    esp_lcd_touch_handle_t tp_handle = NULL; // Declare a handle for the touch panel

    // I2C is already initialized in main.c - no need to initialize again
    ESP_LOGI(RGB_PORT_TAG, "Using pre-initialized I2C bus for CH422G and touch");              

    // Perform LCD hardware reset via CH422G before starting LVGL
    ESP_LOGI(RGB_PORT_TAG, "Performing LCD hardware reset");
    ESP_ERROR_CHECK(waveshare_rgb_lcd_reset()); 

#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911
    ESP_LOGI(RGB_PORT_TAG, "Initialize GPIO");      // Log GPIO initialization
    gpio_init();                           // Initialize GPIO pins
    ESP_LOGI(RGB_PORT_TAG, "Initialize Touch LCD"); // Log touch LCD initialization
    waveshare_esp32_s3_touch_reset();      // Reset the touch panel

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;                                          // Declare a handle for touch panel I/O
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG(); // Configure I2C for GT911 touch controller

    ESP_LOGI(RGB_PORT_TAG, "Initialize I2C panel IO");                                                                          // Log I2C panel I/O initialization
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle)); // Create new I2C panel I/O

    ESP_LOGI(RGB_PORT_TAG, "Initialize touch controller GT911"); // Log touch controller initialization
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,                // Set maximum X coordinate
        .y_max = EXAMPLE_LCD_V_RES,                // Set maximum Y coordinate
        .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST, // GPIO number for reset
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT, // GPIO number for interrupt
        .levels = {
            .reset = 0,     // Reset level
            .interrupt = 0, // Interrupt level
        },
        .flags = {
            .swap_xy = 0,  // No swap of X and Y
            .mirror_x = 0, // No mirroring of X
            .mirror_y = 0, // No mirroring of Y
        },
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp_handle)); // Create new I2C GT911 touch controller
#endif                                                                               // CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911

    ESP_ERROR_CHECK(lvgl_port_init(panel_handle, tp_handle)); // Initialize LVGL with the panel and touch handles

    // Register callbacks for RGB panel events
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
#if EXAMPLE_RGB_BOUNCE_BUFFER_SIZE > 0
        .on_bounce_frame_finish = rgb_lcd_on_vsync_event, // Callback for bounce frame finish
#else
        .on_vsync = rgb_lcd_on_vsync_event, // Callback for vertical sync
#endif
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL)); // Register event callbacks

    return ESP_OK; // Return success
}

/******************************* Reset the LCD display **************************************/
esp_err_t waveshare_rgb_lcd_reset()
{
    ESP_LOGI(RGB_PORT_TAG, "Starting comprehensive LCD reset sequence");
    
    // Configure CH422G to output mode
    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // Perform multiple reset cycles for better timing stability
    for (int cycle = 0; cycle < 3; cycle++) {
        ESP_LOGI(RGB_PORT_TAG, "LCD reset cycle %d", cycle + 1);
        
        // Pull LCD_RST low to reset the LCD (IO3 = 0x08, without it = reset active)
        write_buf = 0x16; // Base value without LCD_RST bit
        i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ets_delay_us(500 * 1000); // Hold reset for 500ms (longer than before)

        // Pull LCD_RST high to complete reset sequence
        write_buf = 0x1E; // Base value with LCD_RST bit (0x16 + 0x08)
        i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        ets_delay_us(300 * 1000); // Wait 300ms between cycles
    }
    
    // Final stabilization delay
    ESP_LOGI(RGB_PORT_TAG, "LCD reset complete, stabilizing...");
    ets_delay_us(1000 * 1000); // Wait 1 second for full stabilization

    return ESP_OK;
}

/******************************* Turn on the screen backlight **************************************/
esp_err_t wavesahre_rgb_lcd_bl_on()
{
    // Configure CH422G to output mode
    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // Pull the backlight pin high to light the screen backlight
    write_buf = 0x1E;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ESP_OK;
}

/******************************* Turn off the screen backlight **************************************/
esp_err_t wavesahre_rgb_lcd_bl_off()
{
    // Configure CH422G to output mode
    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // Turn off the screen backlight by pulling the backlight pin low
    write_buf = 0x1A;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ESP_OK;
}

/******************************* Example code **************************************/
// Demo code removed (lv_demos dependency not included)
