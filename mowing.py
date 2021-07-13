#!/usr/bin/python3

import math
import time
import struct
import datetime           # used by drv_mower

# application imports
import my_queue as que
import my_gendef as gnl
import my_gpio 

if my_gpio.Im_a_Raspberry:
    
    # to access the input pin for the drive motor command feedback
    import RPi.GPIO as GPIO
    
    # setup the I2C bus 
    import smbus        # used by motor_cmd

    # I2C channel used by Rpi (Arduino Motor driver)
    i2c_ch = 1

    # I2C address Arduino Motor driver
    ARD_ADDR = 0x8;	
   
    # setup the I2C bus
    bus = smbus.SMBus(i2c_ch)

    # Give the I2C device time to settle
    time.sleep(2)

# only import the QMC5883L driver on raspberry; Windows 10 has no I2C bus
try:
    import py_qmc5883l
    compass_present = True
    print ('mowing: QMC8553L library passed')
    # set the compass calibration
    sensor = py_qmc5883l.QMC5883L()
    
    # sensor calibration for chp 8553 7008 on 2020/08/07
    sensor.calibration = [[1.0176504703722995, -0.02987973970930191, -137.34567137145933], [-0.029879739709301856, 1.0505821559575423, -2445.845227907534], [0.0, 0.0, 1.0]]
    
except:
    compass_present = False
    print ('mowing: QMC8553L library not present')
    
    

    
# global variables - shared outside this thread
veh_pos_cm = (0,0)          # vehicle position (x,y) in cm

- 0.
# threads
kill_thread_1 = False       # set by toggle_mower_thread()

# global variables - not shared outside this thread
mower_width = 53            # mower width in cm
work_area_limits = []       # list of work area limits per segment;
                            # (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm)

# mowing patterns
mow_traditional = 1
mow_spiral = 2
mow_random = 3            





# get the vehicle position by triangulation
def get_veh_position():

    steps_360 = 48 * 8                  # 48 steps with 8 micro steps per step
    degree_step = 360.0 / steps_360     # based upon 48 * 8 microsteps per 360 degrees
    rot_res_step = 1                    # resolution in steps
    rot_res_deg = rot_res_step * degree_step  
    
    
    # beacon bearings will be the list of beacon the vehicle can see and may change over the course
    beacon_bearings = {}
    all_intersections = {}
    
    # the vehicle is stopped, get it's absolute bearing
    current_bearing = get_compass_bearing()
    
    # rotate the sensor for a full 360 degrees to get all beacons we can see
    rel_bearing = 0
    while(rel_bearing <= 360):
    
        # if the sensor acquired a beacon 
        beacon_acquired = get_sensor(rel_bearing)
        
        # 0 if no beacon was acquired, else beacon bumber
        if beacon_acquired > 0:
            
            # get the beacon number and bearing
            abs_bearing = math.fmod(rel_bearing + current_bearing,360.0)
            beacon_bearings[beacon_acquired] = abs_bearing
            print ("get_veh_pos: bearing_acquired = ", beacon_acquired, "abs_bearing = ", abs_bearing)
            
        # rotate for the next acquistition attempt
        rotate_sensor(rot_res_step, my_gpio.CW)
        rel_bearing += rot_res_deg
        
        
    # determine the best 2 beacons to use, try to get as close to a 90 degree angle bewteen them
    for bcn1 in beacon_bearings:
        for bcn2 in beacon_bearings:
        
            # don't store when same beacon or duplicate 
            if bcn1 < bcn2:
                # calculate the angle between the 2 bearings
                brg_ang = math.fabs(get_bear_diff(beacon_bearings[bcn1], beacon_bearings[bcn2]))
                
                # quality 0 (90 degree angle)is best, quality 90 is worst
                quality = math.fabs(brg_ang - 90.0)
                
                # and store the results so that the best 2 bearings can be picked later
                all_intersections[(bcn1,bcn2)] = (bcn1, bcn2, brg_ang, quality)
    
    # pick the best 2 bearings
    best_quality = 90.0
    for bcn in all_intersections:

        # unpack
        (t_bcn1, t_bcn2, t_brg_ang, t_quality) = all_intersections[bcn]    
        print ("get_veh_pos: brg = ", t_bcn1, t_bcn2, t_brg_ang, t_quality)

        if t_quality < best_quality:
            bcn1 = t_bcn1
            bcn2 = t_bcn2
            brg_ang = t_brg_ang
            quality = t_quality
            
            # found a new best pair
            best_quality = t_quality
            
    print ("get_veh_pos: using beacons = ", bcn1, beacon_bearings[bcn1], bcn2, beacon_bearings[bcn2])
            
    # calculate the position
    veh_pos_cm_tri = update_position([beacon_bearings[bcn1], beacon_bearings[bcn2]])
    
    # rotate the sensor back to the starting position
    rotate_sensor(steps_360, my_gpio.CCW)
    
    return(veh_pos_cm_tri)




# once we have 2 bearings, calculate the position 
def update_position(bearings):

    # temporary beacon locations fore testing
    beacon_pos_cm = [ (100,100), (200,1400), (1450,1450) ]
    
    
    m = {}      # linear equation coefficient
    b = {}      # linear equation offset
    
    # from the absolute bearing, calculate slope 'm' and offset 'b' from y = m.x + b
    for bcn in range(0,2):
    
        # calculate the slope 'm'
        m[bcn] = get_m(bearings[bcn])
        
        # calculate the offset 'b' from the fixed position of the beacon
        (x,y) = beacon_pos_cm[bcn]
        b[bcn] = y - (m[bcn] * x)
    
        print ('update_position: ', beacon_pos_cm[bcn], 'm = ', m[bcn], 'b =', b[bcn])

    # calculate the intersection
    (x, y) = intersect(m[0], b[0], m[1], b[1])  
    print ('update_position: intersection = (', x, ',', y, ')')
    return(x,y)



  
def get_bear_diff(bear1, bear2):
    r = (bear2 - bear1) % 360.0
    # Python modulus has same sign as divisor, which is positive here,
    # so no need to consider negative case
    if r >= 180.0:
        r -= 360.0
    return r
 
 
    
  
def get_m(bearing):

    # adjust to 0-360
    bearing = math.fmod(bearing, 360.0)
    
    # [ 0-180 ] 
    if bearing <= 180:
        m = math.tan(math.radians(90.0 - bearing))
        return (m)
    # < 180 - 360 ]
    elif bearing <= 360:
        m = math.tan(math.radians(270.0 - bearing))
        return (m)
    
        
    
# calculate the intersection of 2 linear equations
def intersect(m1, b1, m2, b2):

    # m1 * x + b1 = m2 * x + b2
    # (m1 - m2)* x = b2 - b1
    # x = (b2 - b1) / m1 - m2)
    x = (b2 -b1) / (m1 - m2)
    
    # and from the y coordinate, calculate the y coordinate from one of the two lines
    y = m1 * x + b1
    print ('intersect: x =', x, 'y = ', y)
    return (x,y)
    
  

# check if the sensor acquired a beacon
def get_sensor(brg):

    sensor = 0
    
    if (brg > 20.0 and brg <21.0):
        sensor = 1
        
    if (brg > 220.0 and brg <221.0):
        sensor = 2
    
    if (brg > 340.0 and brg <341.0):
        sensor = 3
    
    return(sensor)


 
# rotate the camera by the requested number of steps (positive or negative) 
def rotate_sensor(no_steps, direction):

    # delay in seconds for 48 step stepping motor
    delay = .0208 / 8

    if my_gpio.Im_a_Raspberry:
        # set the direction
        GPIO.output(my_gpio.cr_DIR_PIN, direction)
        
        # and rotate for the requestde number of steps
        for x in range(no_steps):
            GPIO.output(my_gpio.cr_STEP_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(my_gpio.cr_STEP_PIN, GPIO.LOW)
            time.sleep(delay)
    
    return()        
        
                

# get the compass bearing from the QMC5883L compass
def get_compass_bearing():

    if compass_present:
        b = sensor.get_bearing()
    else:
        b = 0.0
    return (b)




def get_bear_diff(bearing_1, bearing_2):
    r = (bearing_2 - bearing_1) % 360.0
    # Python modulus has same sign as divisor, which is positive here,
    # so no need to consider negative case
    if r >= 180.0:
        r -= 360.0
    return r
 
 
 
    
# wait until the drive or zero turn command is completed		
def wait_cmd_done():

    # need to sleep for a brief moment to allow the arduino driver to process the cmd
    time.sleep(0.5)
    
    # wait until status indicates completed
    cmd_finished = GPIO.input(my_gpio.cr_DRV_PIN)
    while not cmd_finished:
        #print ('wait_cmd_done: loop cmd_finished = ', cmd_finished)
        time.sleep(0.5)
        cmd_finished = GPIO.input(my_gpio.cr_DRV_PIN)

    return



# turn to the requested bearing and drive
def turn_drive(req_bearing, dist_cm):

    # motor reduction = 60:1, sprockets 1:1, wheel diameter 20 cm
    # 1 step (revolution) = 62.83/60 = 10.47 cm
    steps_cm   = 0.0955         # steps per cm
    
    # get the absolute bearing the mower is pointing to
    #current_bearing = 0        # fixed angle for now
    time.sleep(1)
    current_bearing = get_compass_bearing()
    print ('turn_drive: current_bearing = ', current_bearing)
       
       
       
    # calculate the angle to turn
    turn_angle = get_bear_diff(current_bearing, req_bearing)
    
    # calculate the distance the wheels need to travel
    turn_cm = turn_angle  / 360.0 * math.pi * mower_width
    turn_steps = int(turn_cm * steps_cm)
    
    # calculate the steps to drive
    drive_steps = int(dist_cm * steps_cm)
    
    
    if my_gpio.Im_a_Raspberry:
        
        # zero turn
        mower_cmd('z,' + str(turn_steps) + ',0,50')
        wait_cmd_done()
        
        # drive
        mower_cmd('d,' + str(drive_steps) + ',0,80')
        wait_cmd_done()
        

    else:
        print ('turn_drive: turn_angle = ', turn_angle, 'steps = ', steps)
           
    return
    

# send command to Arduino based motor driver
def mower_cmd(command):

# Raspberry Pi I2C Master to Arduino Slave
# Test sending command to Arduino over I2C bus
#
#   Commands by reading / writing registers
#
#   Register    Command     R/W     Type        Description
#   0                       W/R     Signed Int  Command
#   1,2         Drive       W       Signed Int  No of Steps
#   3,4                     W       Signed Int  Steering steps
#   5,6                     W       Signed Int  Duty
#   7-10        Show/Tune   R/W     Float       Kp
#   11-14                   R/W     Float       Ki
#   15-18                   R/W     Float       Kd
#   19,20       Show        R       Signed Int  Motor 0 count
#   21,22       Show        R       Signed Int  Motor 1 count

    # Arduino Motor driver registers
    reg_cmd       = 0            # 1 bytes, used by drive command
    reg_drive     = 1            # 6 bytes, used by drive command
    reg_tune      = 7            # 12 bytes, used by tune command
    reg_motor     = 19           # 4 bytes, used by show command
    reg_next      = 23           # next available register

    data = ''
    
    print ("motor_cmd: ", command)


    # drive command: d,<steps>,<steer_delta>,<duty>
    if command[:1] == 'd':          # slice first character of command
	
        cmd_ln = command.split(',')
        if len(cmd_ln) < 4:
                print (" Enter d,<steps>,<steer_delta>,<duty>")
        else:
                # setup the drive registers (6 bytes)
                b_reg_drive = bytearray(6)
                
                steps = int(cmd_ln[1])
                struct.pack_into('<h', b_reg_drive, 0, steps)              # short integer 2 bytes

                steer_steps = int(cmd_ln[2])
                struct.pack_into('<h', b_reg_drive, 2, steer_steps)        # short integer 2 bytes

                duty = int(cmd_ln[3])
                struct.pack_into('<h', b_reg_drive, 4, duty)               # short integer 2 bytes
                print (b_reg_drive)

                # convert to list
                l_reg_drive = list(b_reg_drive)

                # Writes 1+6 bytes, first byte is the register address to write into
                bus.write_i2c_block_data(ARD_ADDR, reg_drive, l_reg_drive)

                # Write the drive command; this will be reset by the slave
                cmd_char = 0x64
                list1 = [cmd_char]
                bus.write_i2c_block_data(ARD_ADDR, reg_cmd, list1)
                
    # zero turn command: z,<steps>,0,<duty>
    elif command[:1] == 'z':          # slice first character of command

        cmd_ln = command.split(',')
        if len(cmd_ln) < 4:
                print (" Enter z,<steps>,0,<duty>")
        else:
                # setup the drive registers (6 bytes)
                b_reg_drive = bytearray(6)
                
                steps = int(cmd_ln[1])
                struct.pack_into('<h', b_reg_drive, 0, steps)              # short integer 2 bytes

                steer_steps = 0
                struct.pack_into('<h', b_reg_drive, 2, steer_steps)        # short integer 2 bytes

                duty = int(cmd_ln[3])
                struct.pack_into('<h', b_reg_drive, 4, duty)               # short integer 2 bytes
                print (b_reg_drive)
                
                print ("  z command", steps)
                # convert to list
                l_reg_drive = list(b_reg_drive)

                # Writes 1+6 bytes, first byte is the register address to write into
                bus.write_i2c_block_data(ARD_ADDR, reg_drive, l_reg_drive)

                # Write the zero turn command; this will be reset by the slave
                cmd_char = 0x7A
                list1 = [cmd_char]
                bus.write_i2c_block_data(ARD_ADDR, reg_cmd, list1)
    
    # Read command back: c
    elif command[:1] == 'c':                    # slice first character of command
    
        # read 1 byte containing the command
        l_reg_cmd = bus.read_i2c_block_data(ARD_ADDR, reg_cmd, 1)
        
        # convert from list to bytearray
        b_reg_cmd = bytearray(l_reg_cmd)
        data = struct.unpack('<B', b_reg_cmd)    # unsigned int 1 byte
        print ("cmd data = ", data)
        

    # tune command: t,<Kp>,<Ki>,<Kd>
    elif command[:1] == 't':                    # slice first character of command
	
        cmd_ln = command.split(',')
        if len(cmd_ln) < 4:
                print ("Enter t,<Kp>,<Ki>,<Kd>")
        else:
                # setup the drive registers (12 bytes)
                b_reg_tune = bytearray(12)
                
                kp = float(cmd_ln[1])
                struct.pack_into('<f', b_reg_tune, 0, kp)            # float, 4 bytes

                ki = float(cmd_ln[2])
                struct.pack_into('<f', b_reg_tune, 4, ki)            # float, 4 bytes

                kd = float(cmd_ln[3])
                struct.pack_into('<f', b_reg_tune, 8, kd)            # float, 4 bytes
                print (b_reg_tune)

                # convert to list
                l_reg_tune = list(b_reg_tune)

                # Writes 1+12 bytes, first byte is the register address to write into
                bus.write_i2c_block_data(ARD_ADDR, reg_tune, l_reg_tune)

                # Signal the tune command is ready; this will be reset by the slave
                cmd_char = 0x74
                list1 = [cmd_char]
                bus.write_i2c_block_data(ARD_ADDR, reg_cmd, list1)

        
    # show command: s
    elif command[:1] == 's':                    # slice first character of command
            
        # read 3 x 4 bytes containing the tuning parameters
        l_reg_tune = bus.read_i2c_block_data(ARD_ADDR, reg_tune, 12)
        
        # convert from list to bytearray
        b_reg_tune = bytearray(l_reg_tune)
        data = struct.unpack('<fff', b_reg_tune)    # float, 3 x 4 bytes
        print ("tuning data = ", data)
        
        # let i2c bus recover
        time.sleep(0.1)
        
        # read 2 x 2 bytes containing the motor counters
        l_reg_motor = bus.read_i2c_block_data(ARD_ADDR, reg_motor, 4)
        
        # convert from list to bytearray
        b_reg_motor = bytearray(l_reg_motor)
        data = struct.unpack('<hh', b_reg_motor)    # short integer
        print ("motor_data = ", data)
                
                
    # real all registers command: r
    elif command[:1] == 'r':                    # slice first character of command
    
        # read the entire register back from the Arduino
        data = bus.read_i2c_block_data(ARD_ADDR, reg_cmd, reg_next)
        print("all registers = ", data)
         
    # quit
    elif command == 'q':
        exit()
	
    return(data)



# the work area is given in coordinates in cm. Now calculate the coordinates that
# the centre mower should adhere to due to it's width and the linear equations of each of the segments 
# of the work area
def calc_work_path(work_area_coord_cm):

    work_path = []          # the work path
    
    segments_0 = []         # linear equation data of each of the segments
    segments_1 = []         # linear equation data and end coordinates of each of the segments
    first_coord = None
    first_segment = None
    
    

    # determine the line segments of the work area
    for coord in work_area_coord_cm:
            
        # first time, no segments defined yet
        if not first_coord == None:
        
            # determine the linear equation of this segment
            (m, b_in, mid_coord) = get_lin_eq_segment(last_coord, coord, mower_width/2.0)
            
            # store only the inner boundary and the mid coordinate, which we  need only in case of a vertical segment
            segments_0.append((m, b_in, mid_coord))
        else:
            # first coordinate, store it
            first_coord = coord
            
        # save coord for the next segment
        last_coord = coord
        
    # now create the last segment by connecting to the first coordinates
    (m, b_in, mid_coord) = get_lin_eq_segment(last_coord, first_coord, mower_width/2.0)
    
    # store only the inner boundary
    segments_0.append((m, b_in, mid_coord))        
     
    #print ('calc_work_path: segments_0 = ', segments_0)
    
    
    
    
    # calculate the coordinates where each of the segments intersect
    for segment in segments_0:
    
        # need to unpack to store the segment in work data
        (m, b_in, mid_coord) = segment
    
        # first segment
        if not first_segment == None:
        
            # calculate the coordinates where the segments cross            
            (x,y) = calc_segment_cross(last_segment, segment)
            segments_1.append((x, y, m, b_in))
        else:
        
            # first segment, store it
            first_segment = segment
            
        # save segment
        last_segment = segment
        
           
    # now calculate the crossing of the last segment with the first segment    
    (m, b_in, mid_coord) = first_segment
    (x,y) = calc_segment_cross(last_segment, first_segment)
    segments_1.append((x, y, m, b_in))
    
    #print ('calc_work_path: segments_1 = ', segments_1)
   
   
   
   
    # add the end coordinate of each segment, by copying it from the start coordinate of the next segment
    no_segments = len(segments_1)
    for index, segment in enumerate(segments_1):
 
        # unpack segment_1
        (xs, ys, m, b) = segment
        
        # the start point of the next segment is the end point of this segment
        if index == no_segments - 1:
            (xe, ye, _, _) = segments_1[0]               # the last segment uses the start coord of the first segment
        else:
            (xe, ye, _, _) = segments_1[index + 1]       # all other segments use the start coord of the next segment

        # calculate the absolute bearing and distance in cm to travel
        (abs_brng, trvl_dist_cm) = get_brng(xs, ys, xe, ye)
        
        # now build the final work area data list
        work_path.append((xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm))
    
    #print ('calc_work_path: work_path = ', work_path)
    return (work_path)

    
    

# determine linear equation y = mx + b of line through 2 coordinates (x1, y1) and (x2, y2). Then return 
# the linear equation of the innner boundary. This basically the same line, but offset with the mower's width.
# this funtion assumes a clockwise direction of the mower to determine the offset. Note that the mower
# width can be set to 0. Then also return the coordinate of the mid point.
def get_lin_eq_segment (x1y1, x2y2, offset):

    # unpack
    (x1, y1) = x1y1
    (x2, y2) = x2y2
    
    # check for vertical segments, which would give a divide by 0
    if math.fabs(math.isclose(x1, x2, rel_tol=1e-2)):

        # vertical segment
        if y2 > y1:
            # south to north
            m = float('inf')
            x_mid = (x2 - x1)/2 + x1 + offset
        else:
            # north to south
            m = float('-inf')
            x_mid = (x2 - x1)/2 + x1 - offset
        
        # now calculate the y midpoint on the inner vertical segment
        y_mid = (y2 - y1)/2 + y1
        
        # needs to be initialized, but is not relevant
        b_offset = 0
        
        
        
    else:
    
        # all lines other than vertical
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
        
        # the offset line is y = mx + b plus or minus yw,  where yw is the y-component of the mower width 
        yw = offset / math.cos( math.atan((y2- y1) / (x2 - x1)))
        
        # depending on the line segment direction, add or subtract yw
        if (x2 - x1) > 0:
            b_offset = b - yw
        else:
            b_offset = b + yw
            
        # now calculate the midpoint on the inner segment
        x_mid = (x2 - x1)/2.0 + x1
        y_mid = x_mid * m + b_offset
    
    # return the mid coordinate as a pair
    mid_coord = (x_mid, y_mid)
    
    return (m, b_offset, mid_coord)
 

 
# calculate the coordinate where 2 segments (y = mx + b) intersect 
def calc_segment_cross(seg1, seg2):

    # unpack
    (m1, b1, mid_coord_1) = seg1
    (m2, b2, mid_coord_2) = seg2

    # check for vertical segments, where m is +Inf or -Inf
    if math.isinf(m1):

        # seg1 is the vertical segment, the segments intersect at x1
        (x,y) = mid_coord_1
        y = m2 * x + b2
    
    else:
        if math.isinf(m2):

            # seg2 is the vertical segment, the segments intersect at x2
            (x,y) = mid_coord_2
            y = m1 * x + b1
        
        else:
         
            # neither seg1 or seg is vertical    
            # m1 * x1 + b1 = m2 * x2 + b2
            x = (b2 - b1) / (m1 - m2)
            y = m1 * x + b1
        
    return (x,y)



# get the absolute bearing starting at coordinate (xs,ys) and  ending at (xe,ye) and the distance between
def get_brng(xs, ys, xe, ye):

    # initialize as floats
    brg = -1.0
    distance = -1.0
    
    # calculate the deltas
    dx = xe - xs
    dy = ye - ys
    #print ('dx = ', dx, 'dy = ', dy)
    
    if (math.fabs(dy) < 0.01):
        
        # on the East - West direction
        if (dx > 0.0):
            # going East
            brg = 90.0
        else:
            # going West
            brg = 270.0

    else:  
        
        if (math.fabs(dx) < 0.01):    

            # on the North - South direction
            if (dy > 0.0):
                # going North 
                brg = 0.0
            else:
                # going South
                brg = 180.0

        else:
        
            brg = math.degrees(math.atan(dx/dy))
            
            if dx > 0.0: 
                # 1 - 179
                if dy < 0.0:
                    # for 91 - 180
                    brg = 180.0 + brg
            else:
                # 181 - 359        
                if dy < 0.0:
                    # for 181 - 269
                    brg = 180.0 + brg
                else:
                    # for 271 - 359
                    brg = 360.0 + brg
                
    # calculate the distance between (xs,ys) and (xe,ye)
    distance = math.sqrt(dx * dx + dy * dy)
    
    return (brg, distance)



# check if value is within range [lim1, lim2], limits included    
def in_range(val, lim1, lim2):

    if lim1 > lim2:
        tmp = lim2
        lim2 = lim1
        lim1 = tmp
        
    inrange = (val >= lim1) and (val <= lim2)
    
    return (inrange)




def traditional_pattern(work_area_limits, res_q):
    print ('traditional_pattern: not implemented yet')
    
    return()
    


def spiral_pattern(work_area_limits, res_q):
    
    global veh_pos_cm

    circumf = 0
    last_circumf = float('inf')          # initialize at largest possible int
    total_distance = 0.0                 # distance traveled
    
    # announce spiral patter has started
    msg = "Spiral pattern started at " + str(datetime.datetime.now())
    que.push_queue(res_q, que.t_message, msg)
    
    #print ('spiral_pattern: work_area_limits = ', work_area_limits)
    #print ('spiral_pattern: veh_pos_cm = ', veh_pos_cm)
    
    # calculate the circumference of the work area
    for seg in work_area_limits:
        (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm) = seg
        circumf = circumf + trvl_dist_cm  
        
    # spiral inwards clockwise until the change in circumference passes through 0 and starts to become negative
    delta_circumf = circumf
    while delta_circumf > 0:           
        
        # mow the outline, clockwise
        new_path = []
        for seg in work_area_limits:
            (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm) = seg
            #print ('\n\nspiral_pattern: seg', seg)           
                
            # prepare the path for the next interation
            new_path.append((xs,ys))

            # check the current position by triangulation
            veh_pos_cm_tri = get_veh_position()
            
            # check if we need to drive to the start postion
            (x_veh, y_veh) = veh_pos_cm
            if (x_veh != xs) or (y_veh != ys):


                # get bearing and distance to (xs, ys) from where we are
                (start_brng, start_dist_cm) = get_brng(x_veh, y_veh, xs, ys)
                
                # zero turn and drive to the start position
                print ('spiral_pattern: start position - zero turning to bearing', start_brng)
                turn_drive(start_brng, start_dist_cm)
                total_distance += start_dist_cm
                
                # send the vehicle position to the hmi 
                veh_pos_cm = (xs,ys)
                que.push_queue(res_q, que.t_vehicle_position, veh_pos_cm)
                
            # zero turn to the required bearing to xe, ye
            print ('spiral_pattern: zero turning to bearing', abs_brng)

            # mow the segment (xs,ys) to (xe,ye) 
            print ('spiral_pattern: mowing to ', xs, ys)
            turn_drive(abs_brng, trvl_dist_cm)
            total_distance += trvl_dist_cm
            
            # send the vehicle position to the hmi 
            veh_pos_cm = (xe,ye)
            que.push_queue(res_q, que.t_vehicle_position, veh_pos_cm)
            
        # save the circumference
        last_circumf = circumf
        
        # calculate the new work area limits
        #print ('new_path = ', new_path)
        work_area_limits = []
        work_area_limits = calc_work_path(new_path)   
        
        # calculate the circumference of the new work area
        circumf = 0
        for seg in work_area_limits:
            (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm) = seg
            circumf = circumf + trvl_dist_cm 
        
        # calculate the change in circumference
        delta_circumf = last_circumf - circumf
        print ('spiral_pattern: circumf: ', circumf, delta_circumf)
 
    # announce spiral pattern has completed
    msg = "Spiral pattern completed at " + str(datetime.datetime.now())
    que.push_queue(res_q, que.t_message, msg)
    msg = "Total distance: " + str(total_distance)
    que.push_queue(res_q, que.t_message, msg)
                   
    return()
    
     
    
def random_pattern(work_area_limits, res_q):
    
    global veh_pos_cm
    
    print ('random_pattern')
    
    # mow the outline, clockwise     
    for seg in work_area_limits:
        (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm) = seg
        print ('random_pattern: seg: ', seg)           
            
        # check if we need to drive to the start postion
        (x_veh, y_veh) = veh_pos_cm
        if (x_veh != xs) or (y_veh != ys):
        
            # drive to the start position (xs,ys)
            veh_pos_cm = (xs,ys)
            print ('random_pattern: driving to ', veh_pos_cm)
            
            # send the vehicle position to the hmi 
            que.push_queue(res_q, que.t_vehicle_position, veh_pos_cm)
            time.sleep(2)
            
            
        # zero turn to the required bearing
        print ('random_pattern: zero turning to bearing', abs_brng)
        
        # mow the segment (xs,ys) to (xe,ye)
        veh_pos_cm = (xe,ye)
        print ('random_pattern: mowing to ', veh_pos_cm)
        
        # send the vehicle position to the hmi 
        que.push_queue(res_q, que.t_vehicle_position, veh_pos_cm)
        time.sleep(2)
        
    # get the extremes to determine the approximate centre of the work area
    x_min = int(gnl.p_x_sz * gnl.cm_pix)
    x_max = 0
    y_min = int(gnl.p_y_sz * gnl.cm_pix)
    y_max = 0
    for seg in work_area_limits:
            (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm) = seg
            
            # the extreme minimum
            x_min = min(xs, xe, x_min)
            y_min = min(ys, ye, y_min)
            
            # the extreme maximum
            x_max = max(xs, xe, x_max)
            y_max = max(ys, ye, y_max)
    
    # determine the approximate centre of the poly surface from it's extremes
    xc = int((x_max - x_min) / 2.0 + x_min)
    yc = int((y_max - y_min) / 2.0 + y_min)
    print ('random_pattern: centre = (', xc, ',', yc,')')
    
    # turn towards the centre with a random error
    
    
    return()


 




# the main for thread mowing
def mowing(cmd_q, res_q):
    
    global veh_pos_cm                   # the vehicle position in cm
    global veh_pos_tri_cm               # the vehicle position as determined by triangulation
    global mowing_pattern               # the mowing pattern requested
    global work_area_coord_cm           # the work area coordinates in cm    
    global bus                          # I2C bus
    
    work_area_cm = []
        
    while True:
        
        print ('\n\nmowing: kill_thread_1 = ', kill_thread_1)
                               
        # get any commands from the hmi
        pkgs = que.get_queue(cmd_q)

        for pkg in pkgs:
        
            print ('mowing: pkg received', pkg)
            
            (pkg_time, type, data) = pkg
            now = time.time()
            if now - pkg_time < que.expire_threshold:
        
                # start mowing command from hmi
                if type == que.t_start_mowing:

                    # unpack the start vehicle position, mow pattern and the work area
                    (veh_pos_cm, mowing_pattern, work_area_coord_cm) = data
                    print ('mowing: veh_pos_cm = ', veh_pos_cm, 'mowing_pattern = ', mowing_pattern)
                 
                    
                    # from the given work area, calculate the coordinates and equations that the mower 
                    # should adhere to
                    work_area_limits = calc_work_path(work_area_coord_cm)
                    
                    if mowing_pattern == mow_traditional:             
                        traditional_pattern(work_area_limits, res_q)
                    else:
                        if mowing_pattern == mow_spiral:
                            spiral_pattern(work_area_limits, res_q)
                        else:
                            if mowing_pattern == mow_random:
                                random_pattern(work_area_limits, res_q)
                        
                            
                # update of the vehicle position as detected by trangulation
                if type == que.t_vehicle_position_tri:

                    # unpack the vehicle position as detected by triangulation
                    (veh_pos_tri_cm) = data
                    print ('mowing: veh_pos_tri_cm = ', veh_pos_tri_cm)
        
        
                  
        # check if this thread needs to exit
        if kill_thread_1: 
            
            #  release I2C bus
            bus.close()
            bus = None      
            return()
        
        # sleep for 1 second
        time.sleep(1)
            
                    
    return()
