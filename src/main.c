#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_dsp.h"
#include "ssd1306.h"
#include "tdoa_lut.h" // Include the TDOA lookup table tdoa_lut[360][3] where tdoa_lut[angle][delays]



// ============================================================================
//  HARDWARE PIN DEFINITIONS
// ============================================================================

// I2C Pins (OLED Display)
#define I2C_SCL_PIN 10
#define I2C_SDA_PIN 11
#define I2C_RESET_PIN -1

// Screen dimensions and center anchors
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define CENTER_X      64
#define CENTER_Y      32
#define ARROW_LEN     22

// I2S Pins (Microphones)
#define I2S_SCK_IO       (4)  // Shared Clock
#define I2S_WS_IO        (5)  // Shared Word Select (L/R clock)
#define I2S_SD_MICS_1_2  (6)  // Data pin for Mics 1 & 2
#define I2S_SD_MIC_3 (7)      // Data pin for 3rd Mic

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
    STATE_POST_TRIGGER,    // Trigger hit, Recording the tail end of the sound
    STATE_PROCESSING       // Buffer frozen. Ready for math.
} system_state_t;

system_state_t current_state = STATE_LISTENING;

// Raw sample buffers for I2S
// 256 samples * 4 bytes = 1024 bytes
int32_t raw_samples_master[256]; 
int32_t raw_samples_slave[256];

// Circular buffers for continuous capture
float captured_mic_0[CAPTURE_SIZE];
float captured_mic_120[CAPTURE_SIZE];
float captured_mic_240[CAPTURE_SIZE];

// Linear buffers for DSP processing
float linear_mic_0[CAPTURE_SIZE];
float linear_mic_120[CAPTURE_SIZE];
float linear_mic_240[CAPTURE_SIZE];

int ring_index = 0; 
int post_trigger_count = 0;

// ============================================================================
//  GLOBAL STATE & BUFFERS FOR FFT APPROACH
// ============================================================================

float buffer_fft_0[CAPTURE_SIZE * 2];
float buffer_fft_120[CAPTURE_SIZE * 2];
float buffer_fft_240[CAPTURE_SIZE * 2];
float cross_corr_buffer[CAPTURE_SIZE * 2];


// ============================================================================
//  HARDWARE INITIALIZATION HELPERS
// ============================================================================

void init_microphones_multichannel(i2s_chan_handle_t *master_hdl, i2s_chan_handle_t *slave_hdl) {

    // Allocate Channels

    // Master for Mics 1 & 2
    i2s_chan_config_t chan_cfg_master = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_master, NULL, master_hdl));

    // Slave for Mic 3
    i2s_chan_config_t chan_cfg_slave = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_slave, NULL, slave_hdl));

    // Configure Master (Mics 1 & 2)
    i2s_std_config_t std_cfg_master = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK_IO,       // MASTER GENERATES CLOCK HERE
            .ws   = I2S_WS_IO,        // MASTER GENERATES WS HERE
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD_MICS_1_2,  // Data pin for Mics 1 & 2
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*master_hdl, &std_cfg_master));


    // Configure Slave (Mic 3)
    i2s_std_config_t std_cfg_slave = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK_IO,       // SLAVE READS CLOCK FROM HERE
            .ws   = I2S_WS_IO,        // SLAVE READS WS FROM HERE
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD_MIC_3,     // Data pin for Mic 3
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*slave_hdl, &std_cfg_slave));

    // Enable Channels (Slave first to sync to Master's clock)

    // Slave
    ESP_ERROR_CHECK(i2s_channel_enable(*slave_hdl));

    // Master
    ESP_ERROR_CHECK(i2s_channel_enable(*master_hdl));

    printf("I2S channels initialized successfully.\n");

}

void init_oled_display(SSD1306_t *dev) {

    i2c_master_init(dev, I2C_SDA_PIN, I2C_SCL_PIN, I2C_RESET_PIN);

    ssd1306_init(dev, SCREEN_WIDTH, SCREEN_HEIGHT);

    ssd1306_contrast(dev, 0xff);
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

int calculate_sample_delay(float *linear_a, float *linear_b) {
    float max_dot_product = -99999999.0f;
    int best_shift = 0;
    int analysis_start = (CAPTURE_SIZE / 2) - 100;

    for (int shift = -MAX_PHYSICAL_SHIFT; shift <= MAX_PHYSICAL_SHIFT; shift++) {
        float dot_product = 0;
        float *ptr_a = &linear_a[analysis_start];
        float *ptr_b = &linear_b[analysis_start + shift];

        dsps_dotprod_f32(ptr_a, ptr_b, &dot_product, WINDOW_SIZE);
        
        if (dot_product > max_dot_product) {
            max_dot_product = dot_product;
            best_shift = shift;
        }
    }
    return best_shift;
}



// int calculate_sample_delay_fft(float *buffer_A, float *buffer_B) {
//     // Forward FFTs 
//     dsps_fft4r_fc32(buffer_A, CAPTURE_SIZE);
//     dsps_fft4r_fc32(buffer_B, CAPTURE_SIZE);

//     // Unscramble both buffers to linear frequency order
//     dsps_bit_rev4r_fc32(buffer_A, CAPTURE_SIZE);
//     dsps_bit_rev4r_fc32(buffer_B, CAPTURE_SIZE);

//     // The Math Loop (Conjugate A, Multiply by B, PHAT Normalize)
//     for (int i = 0; i < CAPTURE_SIZE; i++) {
//         int r_idx = i * 2;
//         int i_idx = i * 2 + 1;

//         // Handle DC Bin Separately (i=0)
//         if (i == 0) {
//             cross_corr_buffer[r_idx] = 0.0f; // Array index 0 (Real part of DC)
//             cross_corr_buffer[i_idx] = 0.0f; // Array index 1 (Imag part of DC)
//             continue; // Skip the rest of the math for this bin and move to i = 1
//         }

//         float rA = buffer_A[r_idx];
//         float iA = buffer_A[i_idx];
//         float rB = buffer_B[r_idx];
//         float iB = buffer_B[i_idx];

//         // Complex multiply with Conjugate A: (rA - j*iA) * (rB + j*iB)
//         float cross_real = (rA * rB) + (iA * iB);
//         float cross_imag = (rA * iB) - (iA * rB);

//         // PHAT Trick: Calculate magnitude for normalization
//         float magnitude = sqrtf((cross_real * cross_real) + (cross_imag * cross_imag));

//         if (magnitude > 1e-6f) { // Prevent divide-by-zero on pure silence
//             cross_corr_buffer[r_idx] = cross_real / magnitude;
            
//             // We make the imaginary part negative.
//             // This conjugate acts as the first mathematical step of the Inverse FFT
//             cross_corr_buffer[i_idx] = -cross_imag / magnitude; 
//         } else {
//             cross_corr_buffer[r_idx] = 0.0f;
//             cross_corr_buffer[i_idx] = 0.0f;
//         }
//     }

//     // Run STANDARD Forward FFT (This acts as our IFFT)
//     dsps_fft4r_fc32(cross_corr_buffer, CAPTURE_SIZE);

//     // Unscramble the time-domain result
//     dsps_bit_rev4r_fc32(cross_corr_buffer, CAPTURE_SIZE);

//     // Find the Peak (Time Difference of Arrival)
//     float max_val = -99999999.0f;
//     int max_index = 0;

//     // Scan only the Real parts of the time-domain output
//     for (int i = 0; i < CAPTURE_SIZE; i++) {
//         float val = cross_corr_buffer[i * 2]; 

//         if (val > max_val) {
//             max_val = val;
//             max_index = i;
//         }
//     }

//     // Handle Negative Delays (Wrapping)
//     int delay = max_index;
//     if (delay > CAPTURE_SIZE / 2) {
//         delay -= CAPTURE_SIZE;
//     }

//     return delay;
// }

// ============================================================================
// OLED DISPLAY HELPERS
// ============================================================================

void draw_direction_arrow(SSD1306_t *dev, float angle_deg) {
    // 0. Convert degrees to radians
    // Formula: radians = degrees * (PI / 180)
    float shifted_deg = angle_deg - 90.0f;
    float angle_rad = shifted_deg * (M_PI / 180.0f);

    // 1. Clear the screen buffer before drawing the new frame
    ssd1306_clear_screen(dev, false);

    // 2. Calculate the main arrow tip coordinate
    // FIX: Add the sine component instead of subtracting to properly 
    // orient 0 degrees towards the top of the inverted OLED Y-axis.
    int target_x = CENTER_X + (int)(ARROW_LEN * cosf(angle_rad));
    int target_y = CENTER_Y + (int)(ARROW_LEN * sinf(angle_rad));

    // 3. Draw the main arrow shaft from center to the target tip
    _ssd1306_line(dev, CENTER_X, CENTER_Y, target_x, target_y, false);

    // 4. Calculate arrowhead wings (offset backwards by roughly 145 degrees / 2.5 rads)
    float left_wing_angle = angle_rad + 2.5f;
    float right_wing_angle = angle_rad - 2.5f;
    int wing_len = 7;

    int left_x = target_x + (int)(wing_len * cosf(left_wing_angle));
    // FIX: Apply the same addition fix to the wings so they render in the right direction
    int left_y = target_y + (int)(wing_len * sinf(left_wing_angle)); 

    int right_x = target_x + (int)(wing_len * cosf(right_wing_angle));
    // FIX: Apply the same addition fix to the wings
    int right_y = target_y + (int)(wing_len * sinf(right_wing_angle)); 

    // 5. Draw the arrowhead wings to the screen buffer
    _ssd1306_line(dev, target_x, target_y, left_x, left_y, false);
    _ssd1306_line(dev, target_x, target_y, right_x, right_y, false);

    // 6. Push the updated frame-buffer content to the physical display
    ssd1306_show_buffer(dev);
}


// ============================================================================
//  MAIN AUDIO PROCESSING TASK
// ============================================================================

void i2s_microphone_task(void *pvParameters) {

    // Initialize oled
    SSD1306_t dev;
    init_oled_display(&dev);

    // Initialize I2S
    i2s_chan_handle_t rx_handle_master, rx_handle_slave;
    init_microphones_multichannel(&rx_handle_master, &rx_handle_slave);

    size_t bytes_read_master = 0;
    size_t bytes_read_slave = 0;

    // Initialize the complex FFT tables. 
    // Do this once 
    esp_err_t ret = dsps_fft4r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    
    if (ret != ESP_OK) {
        printf("Fatal Error: Could not initialize DSP FFT tables!\n");
        return;
    }


    while (1) {
        // Read from I2S Master
        esp_err_t res = i2s_channel_read(rx_handle_master, raw_samples_master, sizeof(raw_samples_master), &bytes_read_master, portMAX_DELAY);
        
        // Immediately read from I2S Slave
        esp_err_t res_slave = i2s_channel_read(rx_handle_slave, raw_samples_slave, sizeof(raw_samples_slave), &bytes_read_slave, portMAX_DELAY);

        if (res == ESP_OK && bytes_read_master > 0 && res_slave == ESP_OK && bytes_read_slave > 0) {
            if (bytes_read_master != bytes_read_slave) {
                printf("Warning: Mismatch in bytes read between Master and Slave! Master: %d, Slave: %d\n", bytes_read_master, bytes_read_slave);
            }

            int total_samples = bytes_read_master / sizeof(int32_t);

            // --- STATE: LISTENING ---
            if (current_state == STATE_LISTENING) {
                float energy_0 = calculate_energy(raw_samples_master, total_samples, 1);
                float energy_120 = calculate_energy(raw_samples_master, total_samples, 0);
                float energy_240 = calculate_energy(raw_samples_slave, total_samples, 1);

                if (energy_0 > TRIGGER_THRESHOLD || energy_120 > TRIGGER_THRESHOLD || energy_240 > TRIGGER_THRESHOLD) {
                    current_state = STATE_POST_TRIGGER;
                    post_trigger_count = 0;
                }

                // Fill circular buffer
                for (int i = 0; i < total_samples; i += 2) {
                    captured_mic_0[ring_index] = (float)(raw_samples_master[i+1] >> 8);
                    captured_mic_120[ring_index] = (float)(raw_samples_master[i] >> 8);
                    captured_mic_240[ring_index] = (float)(raw_samples_slave[i+1] >> 8);

                    ring_index = (ring_index + 1) % CAPTURE_SIZE;
                }
            } 
            
            // --- STATE: POST-TRIGGER CAPTURE ---
            else if (current_state == STATE_POST_TRIGGER) {
                for (int i = 0; i < total_samples; i += 2) {
                    captured_mic_0[ring_index] = (float)(raw_samples_master[i+1] >> 8);
                    captured_mic_120[ring_index] = (float)(raw_samples_master[i] >> 8);
                    captured_mic_240[ring_index] = (float)(raw_samples_slave[i+1] >> 8);
                    
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
                    linear_mic_0[dst_idx]  = captured_mic_0[src_idx];
                    linear_mic_120[dst_idx] = captured_mic_120[src_idx];
                    linear_mic_240[dst_idx] = captured_mic_240[src_idx];

                    src_idx = (src_idx + 1) % CAPTURE_SIZE;
                }
                
                // populate the complex buffers for FFT processing (Interleaved Real/Imaginary)
                for (int i = 0; i < CAPTURE_SIZE; i++) {
                    buffer_fft_0[i * 2 + 0] = linear_mic_0[i];  // Real component (Audio)
                    buffer_fft_0[i * 2 + 1] = 0.0f;            // Imaginary component
                    
                    buffer_fft_120[i * 2 + 0] = linear_mic_120[i]; // Real component (Audio)
                    buffer_fft_120[i * 2 + 1] = 0.0f;            // Imaginary component

                    buffer_fft_240[i * 2 + 0] = linear_mic_240[i]; // Real component (Audio)
                    buffer_fft_240[i * 2 + 1] = 0.0f;            // Imaginary component
                }

                // Run Cross-Correlation using both methods for comparison

                int delay_0_120 = calculate_sample_delay(linear_mic_0, linear_mic_120); // tdoa_lut[i][0] is mic 0 to mic 120 delay
                int delay_120_240 = calculate_sample_delay(linear_mic_120, linear_mic_240); // tdoa_lut[i][1] is mic 120 to mic 240 delay
                int delay_240_0 = calculate_sample_delay(linear_mic_240, linear_mic_0); // tdoa_lut[i][2] is mic 240 to mic 0 delay

                // int delay_0_120_fft = calculate_sample_delay_fft(buffer_fft_0, buffer_fft_120); // tdoa_lut[i][0] is mic 0 to mic 120 delay
                // int delay_120_240_fft = calculate_sample_delay_fft(buffer_fft_120, buffer_fft_240); // tdoa_lut[i][1] is mic 120 to mic 240 delay
                // int delay_240_0_fft = calculate_sample_delay_fft(buffer_fft_240, buffer_fft_0); // tdoa_lut[i][2] is mic 240 to mic 0 delay

                // find degree index in LUT that best matches our observed delays
                int best_angle_index = 0;
                int min_error = 999999;
                // int best_angle_index_fft = 0;
                // int min_error_fft = 999999;
                for (int i = 0; i < 360; i++) {
                    int error = abs(tdoa_lut[i][0] - delay_0_120) + abs(tdoa_lut[i][1] - delay_120_240) + abs(tdoa_lut[i][2] - delay_240_0);
                    if (error < min_error) {
                        min_error = error;
                        best_angle_index = i;
                    }
                    // int error_fft = abs(tdoa_lut[i][0] - delay_0_120_fft) + abs(tdoa_lut[i][1] - delay_120_240_fft) + abs(tdoa_lut[i][2] - delay_240_0_fft);
                    // if (error_fft < min_error_fft) {
                    //     min_error_fft = error_fft;
                    //     best_angle_index_fft = i;
                    // }
                }

                draw_direction_arrow(&dev, (float)best_angle_index);                

                // print to console for debugging
                printf("Cross-Correlation Delays (Samples): Mic0-120: %d, Mic120-240: %d, Mic240-0: %d\n", delay_0_120, delay_120_240, delay_240_0);
                // printf("FFT Cross-Correlation Delays (Samples): Mic0-120: %d, Mic120-240: %d, Mic240-0: %d\n", delay_0_120_fft, delay_120_240_fft, delay_240_0_fft);
                printf("Estimated Angle from LUT: %d deg, Error: %d\n", best_angle_index, min_error);
                // printf("Estimated Angle from FFT LUT: %d deg, Error: %d\n", best_angle_index_fft, min_error_fft);


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