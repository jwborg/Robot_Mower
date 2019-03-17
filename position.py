# !/home/pi/.virtualenvs/cv/bin/python

import cv2
import numpy as np
import math
import time


# application imports
import my_queue as que
    
# global variables - shared outside this thread

# threads
kill_thread_0 = False       # set by toggle_position_thread()

# global variables - not shared outside this thread

# camera rotation
cam_rot_clockwise = True        # the rotating direction of the camera
cam_bearing = 0                 # the bearing in degrees of the camera

# beacon detection return codes
no_beacon    = 0
beacon_green = 1
beacon_red   = 2

# beacon data
abs_bear_veh = 0                                # current bearing of vehicle
beacon = 1                                      # the current beacon (1-based)
last_abs_bear_beacon = [ -1.0, -1.0, -1.0]      # last absolute bearing of the beacon
#last_abs_bear_beacon = [ 350.0, 60.0, 120.0]    # last absolute bearing of the beacon
beacon_last_update = [0,0,0]                    # last update time of beacon in seconds since epoch
beacon_fresh = [ '+inf' , '+inf', '+inf' ]      # seconds since last update
fresh_thres = 60.0                              # threshold in seconds to determine beacon freshness

# bearing equation y = m.x + b
m = [0, 0, 0]
b = [0, 0, 0]

# xy position of the 2 intersections
x01 = 0.0
y01 = 0.0
x12 = 0.0
y12 = 0.0
x02 = 0.0
y02 = 0.0

# beacon position, (x,y) in cm within work area
beacons_defined = False
#beacon_pos = np.array([(0,1500),(1500,1500),(1500,0)])
beacon_pos = np.array([(0,0),(0,0),(0,0)])      # (x,y) coordinates of beacons 1, 2 and 3 in cm

# vehicle position
cur_bear_veh = 0                # vehicle bearing in degrees from North on work area
veh_pos_cm = [0,0]              # (x,y) position in cm within square of work area


    
def get_beacon(cap, sensitivity, active_angle):
    
    
    # Logitech 710
    # maximum view angle is 44 degrees
    # resolution is 320 x 240 pixels
    cam_h_pixels = 320
    cam_max_angle = 44
    degree_pix = cam_max_angle / cam_h_pixels    # pixels per horizontal degree
    
    # the width of the line around the found beacon
    pen_width = 3       
    
    # initialize at nothing found
    status = no_beacon
    obj_cam_angle = 0
    
    # get a frame
    _, frame = cap.read()

    # use a filter to blur the image and get rid of noise
    blurred_frame = cv2.GaussianBlur(frame, (5,5), 0)
    
    # convert to HSV colors for easy color detection
    hsv =cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
    
    # tuned color for white as shown by IR camera
    lower_col = np.array([0, 0, 200])
    upper_col = np.array([180,50,255])
    mask = cv2.inRange(hsv, lower_col, upper_col)
    
    # find the contours, will be returned in array contours
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    # determine the area of the contour
    for contour in contours:
        area = cv2.contourArea(contour)
        #print ("area = ", area)
        
        # select only the large contour to get rid of any remaining noise 
        if area > sensitivity:
                 
            # determine the x,y location of the contour in the frame
            m = cv2.moments(contour)
            center = (int(m['m10'] / m['m00']), int(m['m01'] / m['m00']))
            #print ("center = ", center[0])
            
            # calculate the angle from the center line of the contour as seen by the 
            # camera; center[0] is the X coordinate
            obj_cam_angle = degree_pix * (center[0] - cam_h_pixels / 2)
            
            # determine if the contour is in the requested active angle
            if abs(obj_cam_angle) < active_angle:

                color = (0,255,0)       # green colour, inside active angle
                status = beacon_green
            else:
                color = (0,0,255)       # red colour, outside requested active angle
                status = beacon_red
            
            # draw the contours on the original frame; -1 means all contours found
            cv2.drawContours(frame, contour, -1, color, pen_width)
            
            
    # for debugging            
    cv2.imshow("Frame", frame)
       
    return (status, obj_cam_angle)

# rotate the camera by the requested number of steps (positive or negative) 
def rotate_cam(no_steps):

    return()
    

# automatically rotate the camera over 360 degrees by approximately 'incr_angle' and back again. Return the 
# camera bearing relative to the vehicle after each step
def rotate_cam_360(incr_angle):

    global cam_bearing
    global cam_rot_clockwise
    
    max_bearing = 360
    min_bearing = 0
    
    #steps_rev = 48                  # steps per revolution
    steps_rev = 36                  # steps per revolution
    one_step = 360 / steps_rev      # degrees per step
    
    
    # the effective angle to rotate is determined by the stepping motor, so adjust the requested angle to the closest number of steps, with a minimum of 1 step. So the effective angle may differ from the requested angle.
    no_steps = min(round(incr_angle / one_step,0), 1)
    print ('rotate_cam_360: no_steps = ', no_steps)
    
    # rotate the camera by  approximately the calculated number of steps
    if cam_rot_clockwise:
        rotate_cam(no_steps)
        cam_bearing = cam_bearing + no_steps * one_step
    else:
        rotate_cam(no_steps * -1)
        cam_bearing = cam_bearing - no_steps * one_step
    
    # if a change in direction is required
    if (cam_bearing > max_bearing):
        cam_rot_clockwise = False  
    elif (cam_bearing < min_bearing):
        cam_rot_clockwise = True
    
    return ()

    
    
# a beacon was found, determine which beacon and update its bearing
def identify_beacon_found(obj_cam_angle):

    global last_abs_bear_beacon     # last absolute bearing of the beacon
    global beacon_last_update       # last update time of the beacon

    # initialize at no beacon matched
    matched_beacon = -1
    
    # determine the beacon bearing relative to the vehicle
    rel_cam = cam_bearing + obj_cam_angle
    abs_bear_beacon_cur = math.fmod(abs_bear_veh + rel_cam, 360)
    print ('identify_beacon_found: abs_bear_beacon_cur = ', abs_bear_beacon_cur, last_abs_bear_beacon)
    
    # determine which beacon is within range 
    for bcn in range(0,3):   
        
        # for the first time 
        if last_abs_bear_beacon[bcn] > 999.0:
            last_abs_bear_beacon[bcn] = abs_bear_beacon_cur 
            beacon_last_update[bcn] = time.time()           # record the time
            
            # report which beacon was matched
            matched_beacon = bcn + 1
            print ("first", abs_bear_beacon_cur, last_abs_bear_beacon[bcn])
            break
            
        # once there is previous data, the bearing should match closely to force a update 
        if abs(abs_bear_beacon_cur - last_abs_bear_beacon[bcn]) < 2.0:
            last_abs_bear_beacon[bcn] = abs_bear_beacon_cur
            beacon_last_update[bcn] = time.time()           # record the time
            
            # report which beacon was matched
            matched_beacon = bcn + 1
            print ("following", abs_bear_beacon_cur, last_abs_bear_beacon[bcn])
            break
    
    return(matched_beacon)
    
    
# once a beacon is matched, calculate the position from bearings to the 3 beacons
def update_position():

    global m
    global b
    global beacon_fresh
    global x01
    global y01
    global x12
    global y12
    global x02
    global y02
    
    
    # from the absolute bearing, calculate slope 'm' and offset 'b' from y = m.x + b
    for bcn in range(0,3):
    
        # calculate the slope 'm'
        m[bcn] = get_m(last_abs_bear_beacon[bcn])
        
        # calculate the offset 'b' from the fixed position of the beacon
        (x,y) = beacon_pos[bcn]
        b[bcn] = y - (m[bcn] * x)
    
        print ('update_position: ', beacon_pos[bcn], 'm = ', m[bcn], 'b =', b[bcn])

    
    
    # check how recent the time stamp of each beacon is
    now = time.time()
    for bcn in range(0,3):
        beacon_fresh[bcn] = now - beacon_last_update[bcn]
    
        print ('update_position: beacon_fresh[bcn] =', beacon_fresh[bcn])   

    # calculate the 3 intersections
    if beacon_fresh[0] < fresh_thres and beacon_fresh[1] < fresh_thres:
        (x01, y01) = intersect(m[0], b[0], m[1], b[1])
        
    if beacon_fresh[1] < fresh_thres and beacon_fresh[2] < fresh_thres:
        (x12, y12) = intersect(m[1], b[1], m[2], b[2]) 
        xy_12_fresh = max(beacon_fresh[1],beacon_fresh[2])
        
    if beacon_fresh[0] < fresh_thres and beacon_fresh[2] < fresh_thres:
        (x02, y02) = intersect(m[0], b[0], m[2], b[2]) 
        xy_02_fresh = max(beacon_fresh[0],beacon_fresh[2])

    print ('update_position: intersection 0-1 = ', x01, y01, 'intersection 1-2 = ', x12, y12, 'intersection 0-2 = ', x02, y02)

    # determine the freshness of the intersection 01
    xy_01_fresh = max(beacon_fresh[0],beacon_fresh[1])
    if xy_01_fresh < fresh_thres:
    
        # calculate the difference in 'm' to determine sharpness of the angle between the bearings
        bear_01_delta = get_bear_diff(last_abs_bear_beacon[0], last_abs_bear_beacon[1])
        
        # determine the quality of each bearing
        qual_01 = get_quality(xy_01_fresh, bear_01_delta)
    else:
        qual_01 = -1.0
        
    # determine the freshness of the intersection 12
    xy_12_fresh = max(beacon_fresh[1],beacon_fresh[2])
    if xy_12_fresh < fresh_thres:
    
        # calculate the difference in 'm' to determine sharpness of the angle between the bearings
        bear_12_delta = get_bear_diff(last_abs_bear_beacon[1], last_abs_bear_beacon[2])
        
        # determine the quality of each bearing
        qual_12 = get_quality(xy_12_fresh, bear_12_delta)
    else:
        qual_12 = -1.0

    # determine the freshness of the intersection 02
    xy_02_fresh = max(beacon_fresh[0],beacon_fresh[2])
    if xy_02_fresh < fresh_thres:
    
        # calculate the difference in 'm' to determine sharpness of the angle between the bearings
        bear_02_delta = get_bear_diff(last_abs_bear_beacon[0], last_abs_bear_beacon[2])
        
        # determine the quality of each bearing
        qual_02 = get_quality(xy_02_fresh, bear_02_delta)
    else:
        qual_02 = -1.0
        
    print ('qualaties = ', qual_01, qual_12, qual_02)
    
    # now decide which intersection is the best
    best_quality = max(qual_01, qual_12, qual_02)
    if best_quality > -1:
        if qual_01 == best_quality:
            print ('update_position:x01, y01 selected')
            return (x01, y01)
        elif qual_12 == best_quality:
            print ('update_position: x12, y12 selected')
            return (x12, y12)
        elif qual_02 == best_quality:
            print ('update_position: x02, y02 selected')
            return (x02, y02)
        
    # none are good
    return(-1,-1)
  
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
    

# determine the quality of the bearing    
def get_quality(freshness, bear_delta):

    quality = 0
    
    # rate the freshness based upon the least recent bearing
    if freshness <= 10:
        qual_freshness = math.fabs(freshness - 10) * 0.5
    else:
        qual_freshness = 0
        
    # rate the angle between the bearings     
    if (bear_delta > 20 and bear_delta < 160) or (bear_delta > 200 and bear_delta < 340):          
        qual_delta = 5
    else:
        qual_delta = 2
    
    # add both qualities
    quality = qual_freshness + qual_delta
    
    return(quality)
  
 

# discover the position of the beacons sequence
def discover_next_beacon(res_q):

    global cam_bearing
    
    # to set the sensitivity   
    sensitivity = 250       # the area of the beacon image
    active_angle = 10       # the angle in the frame where objects will be reported
    rot_deg = 10            # degrees to rotate the camera by 
    
    
    # do all beacons, but break after each beacon to test if the thread needs to be stopped
    for bcn in range(0,3):
        
        # determine the next unavailable beacon
        if last_abs_bear_beacon[bcn] < 0:
    
            print ('discover_next_beacon: bcn = ', bcn)
            
            # 
            # message the log discovery is started
            type = que.t_message
            data = "Beacon discovery started for beacon " + str(bcn)
            que.push_queue(res_q, type, data)

            # from position 1, rotate slowly until beacon is found
            abs_bear_beacon_cur_1 = last_abs_bear_beacon[bcn]
            while abs_bear_beacon_cur_1 < 0:

                # get a bearing from the videostream, set a small active angle for accuracy
                (status, obj_cam_angle) = get_beacon(cap, sensitivity, active_angle)
                print ('main: obj_cam_angle = ', obj_cam_angle, 'status = ', status)

                # when a beacon is in the valid zone
                if (status == beacon_green):
                
                    # determine the beacon bearing relative to the vehicle
                    rel_cam = cam_bearing + obj_cam_angle
                    abs_bear_beacon_cur_1 = math.fmod(abs_bear_veh + rel_cam, 360)
                    print ('discover_next_beacon: abs_bear_beacon_cur_1 = ', abs_bear_beacon_cur_1)
                    
                    # first bearing from position 1 completed
                    break
                    
                # rotate the camera by 'n' degrees
                rotate_cam_360(rot_deg)

                # if no beacon can be discovered, this is a endless loop. Break
                # out of the loop when the thread needs to be stopped
                if kill_thread_0:
                    return()
                
                # slow the loop down
                time.sleep(1)
        
            # first bearing obtained; drive vehicle forward to position 2
            continue
            
            # from the quadrant determine if we have to scan clock wise or counter clock wise
            if cam_bearing > 180:
                rot_deg = rot_deg * -1
                
            # from position 2, step slowly until beacon is found again
            abs_bear_beacon_cur_2 = last_abs_bear_beacon[bcn]
            while abs_bear_beacon_cur_2 < 0:

                # get a bearing from the videostream, set a small active angle for accuracy
                (status, obj_cam_angle) = get_beacon(cap, sensitivity, active_angle)
                print ('discover_next_beacon: obj_cam_angle = ', obj_cam_angle, 'status = ', status)

                # when a beacon is in the valid zone
                if (status == beacon_green):
                
                    # determine the beacon bearing relative to the vehicle
                    rel_cam = cam_bearing + obj_cam_angle
                    abs_bear_beacon_cur_2 = math.fmod(abs_bear_veh + rel_cam, 360)
                    print ('discover_next_beacon: abs_bear_beacon_cur_2 = ', abs_bear_beacon_cur_2)
                    
                    # second bearing from position 2 completed
                    break
                    
                # rotate the camera by 'n' degrees
                rotate_cam_360(rot_deg)
                
                # if no beacon can be discovered, this is a endless loop. Break
                # out of the loop when the thread needs to be stopped
                if kill_thread_0:
                    return()
        
                # slow the loop down
                time.sleep(1)
        
            # second bearing obtained; drive vehicle back to position 1
            continue

            # calculate the distance between position 1 and 2
            distance = sqrt((x2-x1)**2 + (y2-y1)**2)

            # calculate the beacon position relative to position 1
            (x,y) = rel_pos(distance, abs_bear_beacon_cur_1, abs_bear_beacon_cur_2)
            beacon_pos[bcn] = (x,y)
            
            # update the beacon position on the HMI
            type = que.t_beacon_position
            data = beacon_pos
            que.push_queue(res_q, type, data)
                            

# calculate the position of the beacon relative to position 1 
def rel_pos(d1, A, B):

    #            x     beacon
    #       ----------o
    #       |      / /
    #    d2 |    /  /
    #       |B /   /
    # pos 2 o    /
    #       |   /
    #    d1 |A/
    #       |/
    # pos 1 o  
    
           
        x = d1 / (( 1 / tan(radians(A)))  - (1 / tan(radians(B))))

        d2 = x / tan(radians(B))

        y = d1 + d2
        
        return (x,y)
        

        
# the main for thread get_position
def get_position(cmd_q, res_q):

    global cap
        
    # to set the sensitivity   
    sensitivity = 250       # the area of the beacon image
    active_angle = 10       # the angle in the frame where objects will be reported

    # open the video stream
    cap =cv2.VideoCapture(0)
    if cap.isOpened():

        # set the required resolution
        ret = cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
        ret = cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

        while True:
            
            print ('\n\nkill_thread_0 = ', kill_thread_0)
            
            # get any commands from the hmi
            pkgs = que.get_queue(cmd_q)

            for pkg in pkgs:
                (pkg_time, type, data) = pkg
                now = time.time()
                if now - pkg_time < que.expire_threshold:
            
                    # start vehicle position from hmi
                    if type == que.t_vehicle_start_position:

                        # unpack the vehicle position and convert from pixels to cm
                        (veh_pos_cm, cm_pix) = data
                        print ('get_position: veh_pos_cm = ', veh_pos_cm)
            
            
            # all beacons defined?
            if  not beacons_defined:
            
                discover_next_beacon(res_q)
                
            # the beacons are defined
            else:
            
                # get a bearing from the videostream and 
                (status, obj_cam_angle) = get_beacon(cap, sensitivity, active_angle)
                print ('main: obj_cam_angle = ', obj_cam_angle, 'status = ', status)
                
                # when a beacon is in the valid zone
                if (status == beacon_green):

                    # identify the beacon and update it's bearing
                    matched_beacon = identify_beacon_found(obj_cam_angle)
                    if matched_beacon > 0:
                        print ('matched_beacon = ', matched_beacon)
                    
                        # update the position by triangulation
                        veh_pos_cm = update_position()
                
                        # put the vehicle position on the results queue
                        type = que.t_vehicle_position
                        data = veh_pos_cm
                        que.push_queue(res_q, type, data)
            
                # rotate the camera by 'n' degrees. For testing set at 10.
                rotate_cam_360(10)
                print ('main: cam_bearing = ', cam_bearing)
                      
            # check if this thread needs to exit
            if kill_thread_0:

                # shutdown the video stream    
                cap.release()
                cv2.destroyAllWindows()            
                return()
            
            # sleep for 100 ms. time.sleep has issues with video capture, so use waitkey
            #key = cv2.waitKey(100)
            key = cv2.waitKey(1000)
            
            
    else:
        print ("main: Could not open camera device, exiting")
        
    return()
