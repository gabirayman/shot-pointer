#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_err.h"

// Hardware Pin Definitions
#define I2S_SCK_IO      (4)
#define I2S_WS_IO       (5)
#define I2S_SD_IO       (6)

// INMP441 configuration
#define I2S_SAMPLE_RATE (16000)

void i2s_microphone_task(void *pvParameters) {
    i2s_chan_handle_t rx_handle;

    // --------------------------------------------------------
    // 1. Channel & DMA Configuration
    // --------------------------------------------------------
    // I2S_CHANNEL_DEFAULT_CONFIG automatically sets up DMA descriptors.
    // It configures 6 DMA buffers, each 1000 frames long by default.
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

    // --------------------------------------------------------
    // 2. Standard Mode Configuration
    // --------------------------------------------------------
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        
        // IMPORTANT: The INMP441 is a 24-bit sensor, but it clocks data out 
        // within a 32-bit slot using the Philips I2S standard.
        // .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),

        
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, // INMP441 generates its own internal clock from SCK
            .bclk = I2S_SCK_IO,
            .ws   = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    printf("I2S DMA initialized. Starting read loop...\n");

    // --------------------------------------------------------
    // 3. DMA Read Loop
    // --------------------------------------------------------
    // Allocate a buffer in standard RAM to hold the chunk fetched from DMA
    size_t bytes_to_read = 1024;
    int32_t *raw_samples = (int32_t *)malloc(bytes_to_read);
    size_t bytes_read = 0;

    while (1) {
        // This function blocks until the DMA ring buffer has 'bytes_to_read' available.
        // During this time, the CPU is yielded to other FreeRTOS tasks.
        esp_err_t res = i2s_channel_read(rx_handle, raw_samples, bytes_to_read, &bytes_read, portMAX_DELAY);

        if (res == ESP_OK && bytes_read > 0) {
            int total_samples = bytes_read / sizeof(int32_t);
            
            int32_t max_left = 0;
            int32_t max_right = 0;

            // Loop through the interleaved buffer
            // i increments by 2 to jump from frame to frame
            for (int i = 0; i < total_samples; i += 2) {
                // Extract and shift
                int32_t current_left = raw_samples[i] >> 8;
                int32_t current_right = raw_samples[i+1] >> 8;

                // find the max value in this batch for both channels to get a sense of the amplitude
                if (abs(current_left) > max_left) max_left = abs(current_left);
                if (abs(current_right) > max_right) max_right = abs(current_right);
            }

            printf(">Left_Peak:%ld\n", max_left);
            printf(">Right_Peak:%ld\n", max_right);
            
        }
        
    }
}

void app_main(void) {
    // Pin the mic reading task to Core 1 so it doesn't interrupt standard Core 0 system tasks
    xTaskCreatePinnedToCore(i2s_microphone_task, "i2s_mic_task", 4096, NULL, 5, NULL, 1);
}


























// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/i2c_master.h"
// #include "esp_log.h"
// #include "ssd1306.h"

// // Define the GPIO pin we wired our LED to
// #define BLINK_GPIO GPIO_NUM_6

// // Define the I2C port and pins for the OLED display
// #define OLED_I2C_PORT              I2C_NUM_0    
// #define OLED_SDA_GPIO              GPIO_NUM_5
// #define OLED_SCL_GPIO              GPIO_NUM_4

// static const char *TAG = "main";

// void app_main(void)
// {
//     vTaskDelay(pdMS_TO_TICKS(1000));
//     ESP_LOGI(TAG, "Starting clean 128x32 OLED Test...");

//     // 1. Configure the modern I2C Master Bus config structure
//     i2c_master_bus_config_t bus_config = {
//         .i2c_port = OLED_I2C_PORT,
//         .sda_io_num = OLED_SDA_GPIO,
//         .scl_io_num = OLED_SCL_GPIO,
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true, // Attempt using internal pullups
//     };

//     // Allocate a handle container for our initialized I2C Bus 
//     i2c_master_bus_handle_t bus_handle;
    
//     ESP_LOGI(TAG, "Initializing Native ESP-IDF I2C Master Bus...");
//     esp_err_t bus_ret = i2c_new_master_bus(&bus_config, &bus_handle);
//     if (bus_ret != ESP_OK) {
//         ESP_LOGE(TAG, "I2C Master Bus allocation failed! Error: %d", bus_ret);
//         while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
//     }

//     // 2. Grab the display-specific settings layout from your macro
//     ssd1306_config_t dev_cfg = I2C_SSD1306_128x32_CONFIG_DEFAULT;
//     dev_cfg.i2c_clock_speed = 100000; // Set display speed profile to 100KHz

//     ssd1306_handle_t dev_hdl = NULL;

//     ESP_LOGI(TAG, "Initializing SSD1306 Device Context via Library...");
    
//     // 3. Pass the newly initialized bus handle to the library init function
//     esp_err_t ret = ssd1306_init(bus_handle, &dev_cfg, &dev_hdl);
    
//     if (ret != ESP_OK || dev_hdl == NULL) {
//         ESP_LOGE(TAG, "OLED initialization failed! Verification Code: %d", ret);
//         while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
//     }

//     // Configure display settings
//     ssd1306_set_contrast(dev_hdl, 0xff);
//     ssd1306_clear_display(dev_hdl, false);

//     ESP_LOGI(TAG, "Writing text layout rows...");
//     ssd1306_display_text(dev_hdl, 0, "  TDoA PROJECT  ", false);
//     ssd1306_display_text(dev_hdl, 1, "Status: Testing ", false);
//     ssd1306_display_text(dev_hdl, 2, "Properties Match", false);
//     ssd1306_display_text(dev_hdl, 3, "Phase 2.1 PASS  ", false);

//     /* 1. Reset the GPIO pin to its default state */
//     gpio_reset_pin(BLINK_GPIO);
    
//     /* 2. Configure the GPIO pin direction as an output */
//     gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

//     // Variable to track the current state of the LED
//     uint32_t led_state = 0;

//     while (1) {
//         ESP_LOGI(TAG, "Turning the LED %s", led_state == 1 ? "ON" : "OFF");
        
//         /* 3. Write the state to the physical pin */
//         gpio_set_level(BLINK_GPIO, led_state);
        
//         /* 4. Toggle the state for the next iteration (0 becomes 1, 1 becomes 0) */
//         led_state = !led_state;
        
//         /* 5. Delay the task for 2000 milliseconds (2 seconds) */
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }






































// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "esp_chip_info.h"
// #include "esp_flash.h"
// #include "esp_psram.h"
// #include "esp_heap_caps.h"
// #include "esp_log.h"

// static const char *TAG = "VerificationSystem";

// void app_main(void)
// {
//     ESP_LOGI(TAG, "=================================================");
//     ESP_LOGI(TAG, " ESP32-S3 N16R8 HARDWARE VERIFICATION INITIATED ");
//     ESP_LOGI(TAG, "=================================================");

//     // 1. Probing CPU and Silicon Core Details
//     esp_chip_info_t chip_info;
//     esp_chip_info(&chip_info);
    
//     ESP_LOGI(TAG, "--- CPU and Silicon Specifications ---");
//     ESP_LOGI(TAG, "Processor Architecture: %s", (chip_info.model == CHIP_ESP32S3)? "ESP32-S3" : "Unknown SoC");
//     ESP_LOGI(TAG, "Silicon Core Count:     %d Cores", chip_info.cores);
//     ESP_LOGI(TAG, "Silicon Revision Level: v%d.%d", chip_info.revision / 100, chip_info.revision % 100);
//     ESP_LOGI(TAG, "Radio Transceiver Cap:  %s%s", 
//              (chip_info.features & CHIP_FEATURE_WIFI_BGN)? "Wi-Fi 802.11b/g/n " : "",
//              (chip_info.features & CHIP_FEATURE_BLE)? "Bluetooth LE (v5.0)" : "");

//     // 2. Probing Dynamic Flash Configuration
//     uint32_t flash_size = 0;
//     if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
//         ESP_LOGI(TAG, "--- Flash Memory Specifications ---");
//         ESP_LOGI(TAG, "Dynamically Probed Size: %lu MB (%lu bytes)", flash_size / (1024 * 1024), flash_size);
//     } else {
//         ESP_LOGE(TAG, "Flash Detection Error: Failed to retrieve SPI flash size.");
//     }

//     // 3. Probing External PSRAM Mapping
//     ESP_LOGI(TAG, "--- External Memory (PSRAM) Status ---");
// #if CONFIG_SPIRAM
//     if (esp_psram_is_initialized()) {
//         size_t total_psram_bytes = esp_psram_get_size();
//         size_t allocatable_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
//         ESP_LOGI(TAG, "PSRAM Controller:  Initialized and Active");
//         ESP_LOGI(TAG, "Total Mapped Size: %d MB (%d bytes)", total_psram_bytes / (1024 * 1024), total_psram_bytes);
//         ESP_LOGI(TAG, "Allocatable Heap:  %d bytes", allocatable_psram);
//     } else {
//         ESP_LOGE(TAG, "PSRAM Controller Error: Configured but failed initialization.");
//     }
// #else
//     ESP_LOGW(TAG, "PSRAM Status: Driver disabled (CONFIG_SPIRAM is inactive).");
// #endif

//     // 4. Checking System Heap Allocation Layout
//     size_t internal_sram_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
//     size_t cumulative_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
//     ESP_LOGI(TAG, "--- Memory Allocation Metrics ---");
//     ESP_LOGI(TAG, "Free Internal SRAM (8-bit addressable): %d bytes", internal_sram_free);
//     ESP_LOGI(TAG, "Total Free Memory (SRAM + PSRAM):       %d bytes", cumulative_free);

//     // 5. Execution of High-Baud Memory Coherence Stress Test
// #if CONFIG_SPIRAM
//     if (esp_psram_is_initialized()) {
//         ESP_LOGI(TAG, "--- Commencing MSPI Bus Coherence and Timing Test ---");
        
//         // Allocate a 2 MB array dynamically directly in the Octal PSRAM region
//         size_t test_allocation_size = 2 * 1024 * 1024;
//         ESP_LOGI(TAG, "Allocating %d bytes in external RAM...", test_allocation_size);
        
//         uint32_t *target_buffer = heap_caps_malloc(test_allocation_size, MALLOC_CAP_SPIRAM);
//         if (target_buffer == NULL) {
//             ESP_LOGE(TAG, "Memory Allocation Error: Out of PSRAM space.");
//         } else {
//             ESP_LOGI(TAG, "Allocation Successful. Mapped Address: %p", target_buffer);
            
//             size_t elements_count = test_allocation_size / sizeof(uint32_t);
//             bool verification_success = true;
            
//             // Phase A: Write a high-entropy pseudo-random test pattern
//             ESP_LOGI(TAG, "Writing high-entropy verification patterns...");
//             for (size_t index = 0; index < elements_count; index++) {
//                 target_buffer[index] = (uint32_t)(index ^ 0x5A5A3C3C);
//             }
            
//             // Phase B: Read and verify data coherence
//             ESP_LOGI(TAG, "Reading and verifying memory boundary coherence...");
//             for (size_t index = 0; index < elements_count; index++) {
//                 uint32_t expected_value = (uint32_t)(index ^ 0x5A5A3C3C);
//                 if (target_buffer[index]!= expected_value) {
//                     ESP_LOGE(TAG, "Bus Collision/Data Mismatch at Offset %d! Expected: 0x%08lX, Got: 0x%08lX", 
//                              index, expected_value, target_buffer[index]);
//                     verification_success = false;
//                     break;
//                 }
//             }
            
//             if (verification_success) {
//                 ESP_LOGI(TAG, "Memory Integrity Test Result: PASSED. Zero bit flips detected at 80MHz.");
//             } else {
//                 ESP_LOGE(TAG, "Memory Integrity Test Result: FAILED. High-frequency line interference suspected.");
//             }
            
//             heap_caps_free(target_buffer);
//             ESP_LOGI(TAG, "Test allocation released.");
//         }
//     }
// #endif

//     ESP_LOGI(TAG, "=================================================");
//     ESP_LOGI(TAG, " SYSTEM VERIFICATION CONCLUDED. STATUS: OPERATIONAL ");
//     ESP_LOGI(TAG, "=================================================");

//     while (1) {
//         vTaskDelay(pdMS_TO_TICKS(10000));
//         ESP_LOGI(TAG, "Heartbeat -> Free Internal SRAM: %d bytes | Free PSRAM: %d bytes",
//                  heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
//                  heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
//     }
// }