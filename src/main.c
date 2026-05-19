#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_err.h"

// dsp library
#include "esp_dsp.h"

#include "driver/gpio.h"
// Define the GPIO pin connected to the LED
#define RIGHT_GPIO GPIO_NUM_8
#define LEFT_GPIO GPIO_NUM_9

#include <math.h>

#define CAPTURE_SIZE 1024
// linear buffers for dsp processing
float linear_left[CAPTURE_SIZE];
float linear_right[CAPTURE_SIZE];

#define MAX_PHYSICAL_SHIFT  32     
#define WINDOW_SIZE         512

// Global buffers to hold the frozen snapshot of the sound
float captured_left[CAPTURE_SIZE];
float captured_right[CAPTURE_SIZE];

typedef enum {
    STATE_LISTENING,       // Constantly filling the ring buffer, waiting for a loud noise
    STATE_POST_TRIGGER,    // Trigger hit! Recording the tail end of the sound
    STATE_PROCESSING       // Buffer frozen. Ready for Cross-Correlation.
} system_state_t;

system_state_t current_state = STATE_LISTENING;



// The rolling index for our circular buffer
int ring_index = 0; 
int post_trigger_count = 0;

// Hardware Pin Definitions
#define I2S_SCK_IO      (4)
#define I2S_WS_IO       (5)
#define I2S_SD_IO       (6)

// INMP441 configuration
#define I2S_SAMPLE_RATE (48000)

// -----------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------

// Function to calculate RMS amplitude
float calculate_energy(int32_t *buffer, int num_samples, int channel_offset) {
    float sum_squares = 0;
    // channel_offset: 0 for Left, 1 for Right
    for (int i = 0; i < num_samples; i += 2) {
        float sample = (float)(buffer[i + channel_offset] >> 8);
        sum_squares += (sample * sample);
    }
    return sum_squares / (num_samples / 2);
}



// sample delay using dsp library
int calculate_sample_delay() {
    float max_dot_product = -99999999.0f;
    int best_shift = 0;
    int analysis_start = (CAPTURE_SIZE / 2) - 100;

    for (int shift = -MAX_PHYSICAL_SHIFT; shift <= MAX_PHYSICAL_SHIFT; shift++) {
        float dot_product = 0;

        float *ptr_left = &linear_left[analysis_start];
        // Safely shifted window matching the search range
        float *ptr_right = &linear_right[analysis_start + shift];

        // Hardware-accelerated vector multiplication 
        // Note: dsps_dotprod_f32 does not require a call to dsps_dotprod_init() 
        // because it is a direct math operation. You can remove the init check!
        dsps_dotprod_f32(ptr_left, ptr_right, &dot_product, WINDOW_SIZE);
        
        if (dot_product > max_dot_product) {
            max_dot_product = dot_product;
            best_shift = shift;
        }
    }
    return best_shift;
}

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

    // Threshold for triggering the post-trigger recording state. we will use 4 trillion
    float THRESHOLD = 4e12f;

    // Reset the pins to their default state
    gpio_reset_pin(RIGHT_GPIO);
    gpio_reset_pin(LEFT_GPIO);
    // Set the pins as outputs
    gpio_set_direction(RIGHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEFT_GPIO, GPIO_MODE_OUTPUT);
    // blink at start to indicate we're alive
    for (int i = 0; i < 3; i++) {
                        gpio_set_level(RIGHT_GPIO, 1);
                        gpio_set_level(LEFT_GPIO, 0);
                        vTaskDelay(250 / portTICK_PERIOD_MS);
                        gpio_set_level(RIGHT_GPIO, 0);
                        gpio_set_level(LEFT_GPIO, 1);
                        vTaskDelay(250 / portTICK_PERIOD_MS);
                    }
    gpio_set_level(RIGHT_GPIO, 0);
    gpio_set_level(LEFT_GPIO, 0);

    while (1) {
        // This function blocks until the DMA ring buffer has 'bytes_to_read' available.
        // During this time, the CPU is yielded to other FreeRTOS tasks.
        esp_err_t res = i2s_channel_read(rx_handle, raw_samples, bytes_to_read, &bytes_read, portMAX_DELAY);

        if (res == ESP_OK && bytes_read > 0) {
            int total_samples = bytes_read / sizeof(int32_t);

            if (current_state == STATE_LISTENING) {
                // calculate the RMS (energy) of the current chunk
                float energy_left = calculate_energy(raw_samples, total_samples, 0);
                float energy_right = calculate_energy(raw_samples, total_samples, 1);

                // If the energy exceeds our threshold
                if (energy_left > THRESHOLD || energy_right > THRESHOLD) {
                    current_state = STATE_POST_TRIGGER;
                    post_trigger_count = 0; // reset post-trigger counter
                }

                //either way we save the current chunk into our circular buffer
                for (int i = 0; i < total_samples; i += 2) {
                    captured_left[ring_index] = (float)(raw_samples[i] >> 8);
                    captured_right[ring_index] = (float)(raw_samples[i+1] >> 8);
                    
                    ring_index++;
                    if (ring_index >= CAPTURE_SIZE) {
                        ring_index = 0; // Wrap around!
                    }
                }
            } else if (current_state == STATE_POST_TRIGGER) {
                // We are in the post-trigger state, so we want to keep filling our buffer until we have captured enough tail end audio
                for (int i = 0; i < total_samples; i += 2) {
                    captured_left[ring_index] = (float)(raw_samples[i] >> 8);
                    captured_right[ring_index] = (float)(raw_samples[i+1] >> 8);
                    
                    ring_index++;
                    if (ring_index >= CAPTURE_SIZE) {
                        ring_index = 0; // Wrap around!
                    }
                }

                post_trigger_count += total_samples / 2; // since samples are interleaved stereo, we divide by 2

                if (post_trigger_count >= (CAPTURE_SIZE / 2)) { 
                    current_state = STATE_PROCESSING;
                    printf("Post-trigger capture complete. Buffer frozen for processing.\n");
                }

            } else if (current_state == STATE_PROCESSING) {
                printf("Audio captured! Running Cross-Correlation...\n");

                // Linearize the circular buffer to safely run our math
                int src_idx = ring_index;
                for (int dst_idx = 0; dst_idx < CAPTURE_SIZE; dst_idx++) {
                    linear_left[dst_idx]  = captured_left[src_idx];
                    linear_right[dst_idx] = captured_right[src_idx];
                    
                    src_idx++;
                    if (src_idx >= CAPTURE_SIZE) {
                        src_idx = 0; // Wrap around reading index
                    }
                }

                // Execute the optimized cross-correlation math
                int sample_delay = calculate_sample_delay();

                if (sample_delay > 0) {
                    // turn on right LED
                    gpio_set_level(RIGHT_GPIO, 1);

                    printf("Result: Sound arrived from the RIGHT side.\n");
                    printf("Sample delay: %d samples\n", sample_delay);
                } else if (sample_delay < 0) {
                    gpio_set_level(LEFT_GPIO, 1);
                    printf("Result: Sound arrived from the LEFT side.\n");
                    printf("Sample delay: %d samples\n", sample_delay);
                } else {
                    printf("Result: Sound arrived DEAD CENTER.\n");
                    //alternating fast blink both LEDs for center
                    for (int i = 0; i < 3; i++) {
                        gpio_set_level(RIGHT_GPIO, 1);
                        gpio_set_level(LEFT_GPIO, 0);
                        vTaskDelay(250 / portTICK_PERIOD_MS);
                        gpio_set_level(RIGHT_GPIO, 0);
                        gpio_set_level(LEFT_GPIO, 1);
                        vTaskDelay(250 / portTICK_PERIOD_MS);
                    }
                }
                printf("-----------------------------------\n\n");
                
                // ------------------------------------------
                // section for visualizing the captured audio in the serial plotter
                // ------------------------------------------

                // // ring_index points to the NEXT index to be written to.
                // // This means ring_index is currently pointing at the OLDEST sample in our buffer.
                // int read_idx = ring_index;

                // // Loop through the entire buffer exactly once
                // for (int i = 0; i < CAPTURE_SIZE; i++) {
                    
                //     // Print in Teleplot format: >SeriesName:Value
                //     printf(">Left:%.2f\n", captured_left[read_idx]);
                //     printf(">Right:%.2f\n", captured_right[read_idx]);
                    
                //     // Advance the index and wrap around the circular buffer
                //     read_idx++;
                //     if (read_idx >= CAPTURE_SIZE) {
                //         read_idx = 0;
                //     }

                //     // Optional: A tiny delay to prevent overwhelming the serial buffer
                //     // If your serial monitor drops characters, uncomment the line below.
                //     vTaskDelay(pdMS_TO_TICKS(1)); 
                // }

                // printf("Dump complete. Pausing for 1 second...\n");
                
                // Pause so you can read the terminal, then reset to listen again
                vTaskDelay(pdMS_TO_TICKS(1000));
                // turn off all leds
                gpio_set_level(RIGHT_GPIO, 0);
                gpio_set_level(LEFT_GPIO, 0);
                current_state = STATE_LISTENING;
            }
            
        }
        
    }
}

void app_main(void) {
    // Pin the mic reading task to Core 1 so it doesn't interrupt standard Core 0 system tasks
    xTaskCreatePinnedToCore(i2s_microphone_task, "i2s_mic_task", 4096, NULL, 5, NULL, 1);
}
