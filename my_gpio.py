#!//usr/bin/python3

I2C_CH = 1                 # I2C channel used by Rpi (Arduino Motor driver)
ARD_ADDR = 0x8;	         # I2C address Arduino Motor driver

# compasses
MUX_ADR = 0x70             # TCA9548 I2C Multiplexer I2C ADDR
MUX_BUSES = [5,6,7]
  
# general
CW = 1                     # Clockwise Rotation
CCW = 0                    # Counterclockwise Rotation
ON = 1                     # used for Blade motor
OFF = 0                    

# GPIO pins in BCD mode

# mirror rotation stepping motor
MIR_DIR_PIN = 20           # Direction GPIO Pin
MIR_STEP_PIN = 21          # Step GPIO Pin
MIR_SPR = (200 * 2) * 2      # Steps per Revolution (half steps) * gear ratio

# Hall sensor for mirror rotation syncronisation
MIR_ZERO_PIN = 16

# wheels drive motor command feedback
WHL_DRV_PIN = 18            # wheels drive motor command feedback
         
# collision proximity sensors 
NW_PRX_PIN = 6
N_PRX_PIN  = 5
NE_PRX_PIN = 1
SW_PRX_PIN = 26
S_PRX_PIN  = 19
SE_PRX_PIN = 13

# collision interrupt, any prox sensor fired
PRX_INT_PIN = 23

# blade motor
BLD_MOT_PIN = 12


