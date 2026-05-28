#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_dsp.h"
#include "ssd1306.h"

// ============================================================================
//  HARDWARE PIN DEFINITIONS
// ============================================================================

// I2C Pins (OLED Display)
#define I2C_SCL 10
#define I2C_SDA 11

// I2S Pins (Microphones)
#define I2S_SCK_IO       (4)  // Shared Clock
#define I2S_WS_IO        (5)  // Shared Word Select (L/R clock)
#define I2S_SD_MICS_1_2  (6)  // Data pin for Mics 1 & 2
// FUTURE: #define I2S_SD_MIC_3 (7) // Data pin for 3rd Mic

// ============================================================================
//  AUDIO & DSP CONFIGURATION
// ============================================================================

#define I2S_SAMPLE_RATE     48000
#define CAPTURE_SIZE        1024
#define WINDOW_SIZE         512
#define MAX_PHYSICAL_SHIFT  32     
#define TRIGGER_THRESHOLD   2e12f   // Energy threshold for loud noise

// ============================================================================
//  GLOBAL STATE & BUFFERS
// ============================================================================

typedef enum {
    STATE_LISTENING,       // Constantly filling ring buffer, waiting for noise
    STATE_POST_TRIGGER,    // Trigger hit! Recording the tail end of the sound
    STATE_PROCESSING       // Buffer frozen. Ready for math.
} system_state_t;

system_state_t current_state = STATE_LISTENING;

// Circular buffers for continuous capture
float captured_left[CAPTURE_SIZE];
float captured_right[CAPTURE_SIZE];
// FUTURE: float captured_mic3[CAPTURE_SIZE];

// Linear buffers for DSP processing
float linear_left[CAPTURE_SIZE];
float linear_right[CAPTURE_SIZE];
// FUTURE: float linear_mic3[CAPTURE_SIZE];

int ring_index = 0; 
int post_trigger_count = 0;

// Global display handle
ssd1306_handle_t dev_hdl = NULL;

// ============================================================================
//  GLOBAL STATE & BUFFERS FOR FFT APPROACH
// ============================================================================

float buffer_A[CAPTURE_SIZE * 2];
float buffer_B[CAPTURE_SIZE * 2];
float cross_corr_buffer[CAPTURE_SIZE * 2];

// ============================================================================
//  HARDWARE INITIALIZATION HELPERS
// ============================================================================

void init_oled_display() {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    i2c_master_bus_handle_t i2c_bus_hdl;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus_hdl));

    ssd1306_config_t dev_cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;
    ssd1306_init(i2c_bus_hdl, &dev_cfg, &dev_hdl);

    if (dev_hdl != NULL) {
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_contrast(dev_hdl, 0xFF);
        ssd1306_display_text(dev_hdl, 0, "System Ready", false);
    } else {
        printf("SSD1306 Init Failed\n");
    }
}

i2s_chan_handle_t init_i2s_microphones() {
    i2s_chan_handle_t rx_handle_master;

    // FUTURE: We will add rx_handle_slave here for the 3rd mic

    // Configure I2S Channel (Master for Mics 1 & 2)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle_master));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK_IO,
            .ws   = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD_MICS_1_2,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_master, &std_cfg));
    
    // FUTURE: Init Slave controller here

    // FUTURE: Enable Slave first, then Master to sync clocks
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_master));
    
    printf("I2S initialized successfully.\n");
    return rx_handle_master;
}


// ============================================================================
//  DSP & MATH HELPERS
// ============================================================================

float calculate_energy(int32_t *buffer, int num_samples, int channel_offset) {
    float sum_squares = 0;
    for (int i = 0; i < num_samples; i += 2) {
        float sample = (float)(buffer[i + channel_offset] >> 8);
        sum_squares += (sample * sample);
    }
    return sum_squares / (num_samples / 2);
}

int calculate_sample_delay() {
    float max_dot_product = -99999999.0f;
    int best_shift = 0;
    int analysis_start = (CAPTURE_SIZE / 2) - 100;

    for (int shift = -MAX_PHYSICAL_SHIFT; shift <= MAX_PHYSICAL_SHIFT; shift++) {
        float dot_product = 0;
        float *ptr_left = &linear_left[analysis_start];
        float *ptr_right = &linear_right[analysis_start + shift];

        dsps_dotprod_f32(ptr_left, ptr_right, &dot_product, WINDOW_SIZE);
        
        if (dot_product > max_dot_product) {
            max_dot_product = dot_product;
            best_shift = shift;
        }
    }
    return best_shift;
}

int calculate_sample_delay_fft() {
    // Forward FFTs 
    dsps_fft4r_fc32(buffer_A, CAPTURE_SIZE);
    dsps_fft4r_fc32(buffer_B, CAPTURE_SIZE);

    // Unscramble both buffers to linear frequency order
    dsps_bit_rev4r_fc32(buffer_A, CAPTURE_SIZE);
    dsps_bit_rev4r_fc32(buffer_B, CAPTURE_SIZE);

    // The Math Loop (Conjugate A, Multiply by B, PHAT Normalize)
    for (int i = 0; i < CAPTURE_SIZE; i++) {
        int r_idx = i * 2;
        int i_idx = i * 2 + 1;

        // Handle DC Bin Separately (i=0)
        if (i == 0) {
            cross_corr_buffer[r_idx] = 0.0f; // Array index 0 (Real part of DC)
            cross_corr_buffer[i_idx] = 0.0f; // Array index 1 (Imag part of DC)
            continue; // Skip the rest of the math for this bin and move to i = 1
        }

        float rA = buffer_A[r_idx];
        float iA = buffer_A[i_idx];
        float rB = buffer_B[r_idx];
        float iB = buffer_B[i_idx];

        // Complex multiply with Conjugate A: (rA - j*iA) * (rB + j*iB)
        float cross_real = (rA * rB) + (iA * iB);
        float cross_imag = (rA * iB) - (iA * rB);

        // PHAT Trick: Calculate magnitude for normalization
        float magnitude = sqrtf((cross_real * cross_real) + (cross_imag * cross_imag));

        if (magnitude > 1e-6f) { // Prevent divide-by-zero on pure silence
            cross_corr_buffer[r_idx] = cross_real / magnitude;
            
            // We make the imaginary part negative.
            // This conjugate acts as the first mathematical step of the Inverse FFT
            cross_corr_buffer[i_idx] = -cross_imag / magnitude; 
        } else {
            cross_corr_buffer[r_idx] = 0.0f;
            cross_corr_buffer[i_idx] = 0.0f;
        }
    }

    // Run STANDARD Forward FFT (This acts as our IFFT)
    dsps_fft4r_fc32(cross_corr_buffer, CAPTURE_SIZE);

    // Unscramble the time-domain result
    dsps_bit_rev4r_fc32(cross_corr_buffer, CAPTURE_SIZE);

    // Find the Peak (Time Difference of Arrival)
    float max_val = -99999999.0f;
    int max_index = 0;

    // Scan only the Real parts of the time-domain output
    for (int i = 0; i < CAPTURE_SIZE; i++) {
        float val = cross_corr_buffer[i * 2]; 

        if (val > max_val) {
            max_val = val;
            max_index = i;
        }
    }

    // Handle Negative Delays (Wrapping)
    int delay = max_index;
    if (delay > CAPTURE_SIZE / 2) {
        delay -= CAPTURE_SIZE;
    }

    return delay;
}


// ============================================================================
//  MAIN AUDIO PROCESSING TASK
// ============================================================================

void i2s_microphone_task(void *pvParameters) {
    init_oled_display();
    i2s_chan_handle_t rx_handle = init_i2s_microphones();

    size_t bytes_to_read = 1024;
    int32_t *raw_samples = (int32_t *)malloc(bytes_to_read);
    size_t bytes_read = 0;

    ssd1306_clear_display(dev_hdl, false);
    ssd1306_display_text(dev_hdl, 0, "Listening...", false);

    // Initialize the complex FFT tables. 
    // Do this ONCE at boot before running any acoustic calculations
    esp_err_t ret = dsps_fft4r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    
    if (ret != ESP_OK) {
        printf("Fatal Error: Could not initialize DSP FFT tables!\n");
        return;
    }

    while (1) {
        // Read from I2S Master
        esp_err_t res = i2s_channel_read(rx_handle, raw_samples, bytes_to_read, &bytes_read, portMAX_DELAY);
        
        // FUTURE: Immediately read from I2S Slave here (timeout 0)

        if (res == ESP_OK && bytes_read > 0) {
            int total_samples = bytes_read / sizeof(int32_t);

            // --- STATE: LISTENING ---
            if (current_state == STATE_LISTENING) {
                float energy_left = calculate_energy(raw_samples, total_samples, 0);
                float energy_right = calculate_energy(raw_samples, total_samples, 1);

                if (energy_left > TRIGGER_THRESHOLD || energy_right > TRIGGER_THRESHOLD) {
                    current_state = STATE_POST_TRIGGER;
                    post_trigger_count = 0;
                }

                // Fill circular buffer
                for (int i = 0; i < total_samples; i += 2) {
                    captured_left[ring_index] = (float)(raw_samples[i] >> 8);
                    captured_right[ring_index] = (float)(raw_samples[i+1] >> 8);
                    
                    ring_index = (ring_index + 1) % CAPTURE_SIZE;
                }
            } 
            
            // --- STATE: POST-TRIGGER CAPTURE ---
            else if (current_state == STATE_POST_TRIGGER) {
                for (int i = 0; i < total_samples; i += 2) {
                    captured_left[ring_index] = (float)(raw_samples[i] >> 8);
                    captured_right[ring_index] = (float)(raw_samples[i+1] >> 8);
                    
                    ring_index = (ring_index + 1) % CAPTURE_SIZE;
                }

                post_trigger_count += total_samples / 2;

                if (post_trigger_count >= (CAPTURE_SIZE / 2)) { 
                    current_state = STATE_PROCESSING;
                }
            } 
            
            // --- STATE: PROCESSING & MATH ---
            else if (current_state == STATE_PROCESSING) {
                // Linearize buffer
                int src_idx = ring_index;
                for (int dst_idx = 0; dst_idx < CAPTURE_SIZE; dst_idx++) {
                    linear_left[dst_idx]  = captured_left[src_idx];
                    linear_right[dst_idx] = captured_right[src_idx];
                    
                    src_idx = (src_idx + 1) % CAPTURE_SIZE;
                }
                
                // populate the complex buffers for FFT processing (Interleaved Real/Imaginary)
                for (int i = 0; i < CAPTURE_SIZE; i++) {
                    buffer_A[i * 2 + 0] = linear_left[i];  // Real component (Audio)
                    buffer_A[i * 2 + 1] = 0.0f;            // Imaginary component
                    
                    buffer_B[i * 2 + 0] = linear_right[i]; // Real component (Audio)
                    buffer_B[i * 2 + 1] = 0.0f;            // Imaginary component
                }

                // Run Cross-Correlation
                int sample_delay = calculate_sample_delay();
                int sample_delay_fft = calculate_sample_delay_fft();

                char display_buf[32];
                snprintf(display_buf, sizeof(display_buf), "Delay: %d", sample_delay);

                // Update OLED based on results
                ssd1306_clear_display(dev_hdl, false);
                if (sample_delay > 0) {
                    ssd1306_display_text(dev_hdl, 0, "Right ->", false);
                } else if (sample_delay < 0) {
                    ssd1306_display_text(dev_hdl, 0, "Left <-", false);
                } else {
                    ssd1306_display_text(dev_hdl, 0, "Center", false);
                }
                ssd1306_display_text(dev_hdl, 2, display_buf, false);
                snprintf(display_buf, sizeof(display_buf), "FFT Delay: %d", sample_delay_fft);
                ssd1306_display_text(dev_hdl, 4, display_buf, false);


                // Pause to read, then reset
                // vTaskDelay(pdMS_TO_TICKS(1000));
                // ssd1306_display_text(dev_hdl, 4, "Listening...", false);
                current_state = STATE_LISTENING;
            }
        }
    }
}

// ============================================================================
//  MAIN ENTRY POINT
// ============================================================================

void app_main(void) {
    // Pin the mic reading task to Core 1 so it doesn't interrupt Core 0 system tasks
    xTaskCreatePinnedToCore(i2s_microphone_task, "i2s_mic_task", 8192, NULL, 5, NULL, 1);
}