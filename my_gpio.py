#!//usr/bin/python3

# general
Im_a_Raspberry = False      # Initial state
CW = 1                      # Clockwise Rotation
CCW = 0                     # Counterclockwise Rotation

# GPIO pins in BCD mode

# mirror rotation stepping motor
cr_DIR_PIN = 20             # Direction GPIO Pin
cr_STEP_PIN = 21            # Step GPIO Pin
cr_SPR = 200 * 2            # Steps per Revolution (half steps)

# proximity sensor for mirror rotation syncronisation
mirror_prox_pin = 16

# wheels drive motor command feedback
cr_DRV_PIN = 18             # wheels drive motor command feedback
         


