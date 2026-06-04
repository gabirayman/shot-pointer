import math

# --- Physical Constants ---
SPEED_OF_SOUND = 343.0      # meters per second
SAMPLE_RATE = 48000.0       # Hz
SIDE_LENGTH = 0.22          # 22 cm between mics

# In an equilateral triangle, the distance from the center to any mic (Radius) is:
# R = Side / sqrt(3)
RADIUS = SIDE_LENGTH / math.sqrt(3) 

# --- Microphone Coordinates (X, Y) ---
# 0 degrees is UP (Y-axis), 90 degrees is RIGHT (X-axis)
mics = {
    1: (RADIUS * math.sin(math.radians(0)),   RADIUS * math.cos(math.radians(0))),
    2: (RADIUS * math.sin(math.radians(120)), RADIUS * math.cos(math.radians(120))),
    3: (RADIUS * math.sin(math.radians(240)), RADIUS * math.cos(math.radians(240)))
}

def calculate_lut():
    print("const int8_t tdoa_lut[360][3] = {")
    
    for angle in range(360):
        # Place a virtual sound source 1000 meters away (Far-Field approximation)
        sx = 1000.0 * math.sin(math.radians(angle))
        sy = 1000.0 * math.cos(math.radians(angle))
        
        # Calculate straight-line distance from source to each mic
        d1 = math.sqrt((sx - mics[1][0])**2 + (sy - mics[1][1])**2)
        d2 = math.sqrt((sx - mics[2][0])**2 + (sy - mics[2][1])**2)
        d3 = math.sqrt((sx - mics[3][0])**2 + (sy - mics[3][1])**2)
        
        # Calculate time difference in seconds, then convert to samples
        # Delay = (Dist A - Dist B) / Speed of Sound * Sample Rate
        delay12 = round(((d2 - d1) / SPEED_OF_SOUND) * SAMPLE_RATE)
        delay23 = round(((d3 - d2) / SPEED_OF_SOUND) * SAMPLE_RATE)
        delay31 = round(((d1 - d3) / SPEED_OF_SOUND) * SAMPLE_RATE)
        
        # Print in C array format
        print(f"    {{{delay12:3}, {delay23:3}, {delay31:3}}}, // {angle} deg")
        
    print("};")

if __name__ == "__main__":
    calculate_lut()