string lighting_name
uint8 r
uint8 g
uint8 b
std_msgs/Float32 brightness # sets the brightness in range of 0..1 (0%..100%)
uint8 mode
# use constants below for field mode
uint8 OFF = 0
uint8 DIM = 1
uint8 FLASH = 2
uint8 PULSATION = 3
uint8 ROTATION = 4
uint8 RUNNING = 5