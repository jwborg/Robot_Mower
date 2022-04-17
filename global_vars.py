#!/usr/bin/python3

# global variables - shared outside this thread
veh_pos_cm = (0,0,-1)       # vehicle position (x,y) in cm and vehicle bearing

# threads
t1 = None                   # thread 1
kill_thread_1 = False       # set by toggle_mower_thread()



# global variables within mowing thread - not shared outside this thread
debug_level = 0
version = 1.0
mower_width = 49            # mower width in cm
#work_area_limits = []       # list of work area limits per segment;
                            # (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm)
beacon_pos_cm = []          # beacon positions in cm
view_angle = 36             # the camera view angle in degrees for the 8mm lens
mir_rot_angle = 24          # the angle the mirror rotates, so 15 images per revolution
                            # this is to allow overlap of apriltags on the boundery
ram_disk = '/mnt/ramdisk/'  # use a ram disk to save the SD card
    

# sizing of the beacon sections
no_start_trans  = 12            # number of equal transitions in start sequence 
no_data_trans   = 11            # number of transitions in beacon identification plus 1 parity 
no_trail_trans  = 1             # the last band is an always-there trailer

# mowing patterns, needs to come from mower module
mow_pattern = 1
mow_traditional = 1
mow_spiral = 2
mow_random = 3        



# global variables within main - not shared with threads

# beacons
max_no_bcn = 32                                 # maximum number of beacons supported
sv_beacons = []                                 # screen variables for beacons
b_set_veh_start_pos = False                     # state of the set vehicle button
beacon_pos = []                                 # (beacon_no,x,y) coordinates of beacons in cm
h_beacon_pos = [None] * max_no_bcn              #  handles to maxium 32 beacon positions on canvas

    
# work area
b_set_work_area = False
work_area_coord = []                            # (x,y) coordinates of work area in pixels
work_area_coord_cm = []                         # (x,y) coordinates of work area in cm
last_work_area_coord = []                       # (x,y) coordinates of last work arae in pixels

# starting and stopping of threads
b_start_mowing = False                          # state of mowings stop/start button
 
# vehicle
veh_pos = (0,0,-1)                              # vehicle position (x,y, veh_brng) in pixels; (0,0): lower left corner, -1: not know yet
veh_color = 'red'
h_veh_pos = None                                # handle to vehicle object on canvas
sv_veh_pos = None                               # string variable vehicle position

# grid
b_grid = False                                  # state of the grid on the canvas

# tracing
b_trace = False                                 # tracing state
