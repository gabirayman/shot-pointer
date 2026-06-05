ESP32-S3 Real-Time Acoustic Source Localization
===============================================

This repository contains the firmware for a real-time acoustic source localization system built on the ESP-IDF framework. Utilizing an ESP32-S3 and an array of three I2S MEMS microphones, the system detects impulsive sounds (like claps or impacts), calculates the Time Difference of Arrival (TDOA) using cross-correlation, and points to the sound's origin on an SSD1306 OLED display.

Features
--------

*   **Continuous Circular Buffering:** Constantly samples audio into a ring buffer without blocking, ensuring the exact start of a transient sound is never missed.
    
*   **Spike Triggering:** Monitors energy levels across all microphones. Upon passing a predefined energy threshold, it captures a post-trigger window to perfectly center the impulsive sound for DSP analysis.
    
*   **Multichannel I2S Sync:** Uses both I2S peripherals on the ESP32-S3 (Master and Slave roles) sharing Word Select and Bit Clock lines to maintain phase synchronization across three microphones.
    
*   **TDOA Cross-Correlation:** Linearizes the captured transient and utilizes the esp\_dsp library to perform high-speed vector dot-products, finding the exact sample delay between microphone pairs.
    
*   **LUT-Based Angle Estimation:** Compares the measured sample delays against a precomputed 360-degree Look-Up Table (tdoa\_lut.h) to find the angle with the minimum error.
    
*   **Real-Time OLED Rendering:** Draws a computationally efficient, dynamically rotated arrow on a 128x64 I2C OLED display to indicate the direction of the sound.
    

Hardware Requirements
---------------------

*   **Microcontroller:** ESP32-S3 (I used ESP32-S3 N16R8)
    
*   **Microphones:** 3x I2S MEMS Microphones (e.g., INMP441) arranged in an equidistant circular array.
    
*   **Display:** SSD1306 OLED Display (128x64)
    

Future Roadmap
--------------

*   **FFT / GCC-PHAT Implementation:** An alternative frequency-domain cross-correlation approach (Generalized Cross-Correlation with Phase Transform) is currently sketched in the codebase but commented out. Future updates will implement this to better handle reverberant environments and background noise.
    
*   **Dynamic Thresholding:** Implement a rolling average for the TRIGGER\_THRESHOLD to adapt to varying ambient noise floors.
