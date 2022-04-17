#!/usr/bin/python3

import sys
import math
import time
import struct
import datetime           # used by spiral_pattern

import numpy as np
import cv2 as cv2
from picamera import PiCamera
import apriltag

# application imports
import global_vars as gv
import my_queue as que
import my_gendef as gnl
import my_gpio 

from dataclasses import dataclass 
from typing import List, Tuple


@dataclass
class Point:            # Point (x,y) 
    x: float            # X coordinate
    y: float            # Y coordinate

    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y
        
@dataclass
class Beacon:   
    bcn_no: int         # the beacon number
    a: float            # offset X axis
    b: float            # offset Y axis
    r: float            # radius
    
@dataclass
class BeaconIntersections:
    bcn_a: int         # beacon A (lower beacon number than bcn_b)
    bcn_b: int         # beacon B
    is1: Point         # Intersection point 1 X and Y coordinates
    is2: Point         # Intersection point 2 X and Y coordinates

@dataclass
class TriLateration:
    b1: int            # beacon 1
    b2: int            # beacon 2
    b3: int            # beacon 3
    isp1: Point        # Intersection point 1 X and Y coordinates
    isp2: Point        # Intersection point 2 X and Y coordinates
    isp3: Point        # Intersection point 3 X and Y coordinates
    d123: float        # linear approximation of the intersection circumferance
    centroid: Point    # centroid point of isp1, isp2, isp3 triangle

    def __init__(self, b1, b2, b3, isp1, isp2, isp3, d123, centroid) -> None:
        self.b1 = b1
        self.b2 = b2
        self.b3 = b3
        self.isp1 = isp1
        self.isp2 = isp2
        self.isp3 = isp3        
        self.d123 = d123
        self.centroid = centroid


try:
    # to access the input pin for the drive motor command feedback
    import RPi.GPIO as GPIO
    my_gpio.Im_a_Raspberry = True
    print ('mowing() import RPiGPIO ok')
    
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
    
except:
    my_gpio.Im_a_Raspberry = False
    print ('mowing() RPiGPIO not found')
    


# only import the QMC5883L driver on raspberry; Windows 10 has no I2C bus
try:
    import py_qmc5883l
    compass_present = True
    print ('mowing() QMC8553L library passed')
    # set the compass calibration
    sensor = py_qmc5883l.QMC5883L()
    
    # sensor calibration for chp 8553 7008 on 2021/07/08 (installed on mower)
    sensor.calibration = [[1.0936087442096625, 0.02757899075757958, 1449.4744790188254], [0.027578990757579636, 1.0081253171125029, 204.2978922886598], [0.0, 0.0, 1.0]]
    
except:
    compass_present = False
    print ('mowing() QMC8553L library not present')
    
 

# initialize the camera; it needs 2 seconds to start-up; calibrating the mirror will do that
# the PiCamera library allows setting the camera up in advance resulting in faster images
camera = PiCamera()
frame_size = 1920                                # camera will correct; should be multiple of 32 (horizontal) and 16 vertical
image_size = frame_size * frame_size
camera.resolution = (frame_size, frame_size)     # capture a square as the image will rotate due to the mirror
camera.framerate_range = (1,30)                  # allow from 1 fps to 30 fps for longer shutter speeds




# return the distance to the april tag based upon the measured vertical rib of the tag
def april_distance(v_rib: float) -> float:

    # based upon 17.7 cm vertical rib apriltag
    # the distance calculated from the vertical rib of the April tag is not linear.
    # Calculated by Excel regression: dist = (v_rib / 45499)^(1/-0.958)
    A = 45499.0
    B = -1.04384
    dist = (v_rib / A) ** B

    return (dist)
    




# get the vehicle position 
def get_veh_position(bcn_data : dict) -> Tuple[bool, TriLateration]:

    trilat = TriLateration(0, 0, 0, Point(0.0, 0.0), Point(0.0, 0.0), Point(0.0, 0.0), 0.0, Point(0.0, 0.0))
    best_trilat = TriLateration(0, 0, 0, Point(0.0, 0.0), Point(0.0, 0.0), Point(0.0, 0.0), 0.0, Point(0.0, 0.0))
                                            
    my_name = 'get_veh_position()'

    view_angle = 36             # the camera view angle in degrees for the 8mm lens    
    
    # initialize
    position_found = False
    veh_pos_cm_tri = Point(0,0)
    found_bcn_list = []     # found beacon list for this request
    
    # rotate the mirror in whole steps for 1 full rotation
    step_incr = int(view_angle / 360 * my_gpio.cr_SPR)          # keep as an integer
    no_steps = 0
    rel_mirror_brng = 0 
    while(no_steps < my_gpio.cr_SPR):
        
        # sleep to stabilize from vibrations
        time.sleep(0.2)
        
        # capture single frame directy into opencv object
        frame = np.empty((frame_size * frame_size * 3,), dtype=np.uint8)
        camera.capture(frame, 'bgr')
        frame = frame.reshape((frame_size, frame_size, 3))
        
        # save the settings
        hhmmss = datetime.datetime.now().strftime("%I%M%S")
        
        # for debugging
        #show_camera_settings(hhmmss)
        #fn = ram_disk + hhmmss + 'img_col_' + str(rel_mirror_brng) + '.jpg'
        #cv2.imwrite(fn, frame) 

        # grab the dimensions of the image and calculate the center of the frame
        (h, w) = frame.shape[:2]
        (cX, cY) = (w // 2, h // 2)
        
        # the frame is rotated due to the mirror rotation, de-rotate it to face up
        camera_orientation = -90             # camera points East in reality
        corr_angle =  brng_add(rel_mirror_brng, camera_orientation) * -1
        #print (f'{my_name}: corr_angle = {corr_angle}')
        M = cv2.getRotationMatrix2D((cX, cY), corr_angle, 1.0)
        frm_rotated = cv2.warpAffine(frame, M, (w, h))
        
        # flip the image around the y-axis due to the mirror effect
        frm_flipped = cv2.flip(frm_rotated, 1)
        
        # save the rotated color image for evaluation
        fn = gv.ram_disk + hhmmss + 'img_rot_' + str(rel_mirror_brng) +'.jpg'
        cv2.imwrite(fn, frm_flipped) 
        
        # use a filter to blur the image and get rid of noise
        # the filter size must be odd and a function of the resolution
        frm_blur = cv2.GaussianBlur(frm_flipped, (9,9), 0)

        # convert to HSV colors for easy color detection
        frm_hsv =cv2.cvtColor(frm_blur, cv2.COLOR_BGR2HSV)

        # tuned color for cyan which is not abundant in nature
        # HSV - Hue Saturation Value 0-180, 0-255, 0-255.
        lower_col = np.array([90, 50, 0])       # 90, 50, 0
        upper_col = np.array([110,255,255])
        mask = cv2.inRange(frm_hsv, lower_col, upper_col)
        fn = gv.ram_disk + hhmmss + 'img_msk_' + str(rel_mirror_brng) +'.jpg'
        cv2.imwrite(fn, mask)
        
        # define the AprilTags detector options and then detect the AprilTags in the input image
        options = apriltag.DetectorOptions(families="tag25h9")
        detector = apriltag.Detector(options)
        results = detector.detect(mask)
        
        print(f'{my_name} total AprilTags detected {(len(results))}')
        print(f'results = {results}')
        
        # there may be multiple apritags detected
        for r in results:
            
            print(f'{my_name} tag_id = {r.tag_id}')
            
            # calculate the distance of the 2 vertical ribs of the 4 corners
            (ptA, ptB, ptC, ptD) = r.corners
            x1 = ptD[0] - ptA[0]
            y1 = ptD[1] - ptA[1]
            v1 = math.sqrt(x1*x1 + y1*y1)
            
            x2 = ptC[0] - ptB[0]
            y2 = ptC[1] - ptB[1]
            v2 = math.sqrt(x2*x2 + y2*y2)
                    
            # take the largest vertical rib
            print(f'{my_name} v1= {v1} pixels v2= {v2} pixels')
            v = max(v1, v2)
            
            #  calculate the distance based upon the vertical rib
            distance = april_distance(v)
            print(f'{my_name} distance = {distance} cm')
        
            # and add the apriltag number to the beacon table
            found_bcn_list.append(r.tag_id)
            bcn_data[r.tag_id].r = distance
        
        
        # rotate the mirror
        rotate_cam(step_incr, my_gpio.CW)
        no_steps = no_steps + step_incr
                
        # calculate the angle 
        rel_mirror_brng = int(no_steps / my_gpio.cr_SPR * 360.0)
        #print (f'{my_name}: rotating no_steps = {no_steps} rel_mirror_brng = {rel_mirror_brng}')
    
    # full rotation is required, correct in case it was under 
    step_makeup = my_gpio.cr_SPR  - (my_gpio.cr_SPR // step_incr) * step_incr # '//' is integer division
    if step_makeup > 0:
        #print (f'{my_name}: making up steps for full rotation = {step_makeup}')
        rotate_cam(step_makeup, my_gpio.CW)
    
    # print debug data
    print (f'{my_name}: found_bcn_list = {found_bcn_list}')
    print (f'{my_name}: bcn_data = {bcn_data}')
    
    # find which beacons can be trilaterated
    trilats = BcnTriLaterate(found_bcn_list, bcn_data)

    if len(trilats) > 0:

        position_found = True
        print (f'{my_name} trilats = {trilats}')
    
        # now determine the best beacon
        d123_min = sys.float_info.max
        for trilat in trilats:
            if trilat.d123 < d123_min:
                d123_min = trilat.d123
                best_trilat = trilat
        
    return(position_found, best_trilat)




def TwoBeaconIntersection(c1: Beacon, c2: Beacon, i: BeaconIntersections) -> bool:

    #
    # http://www.ambrsoft.com/TrigoCalc/Circles2/circle2intersection/CircleCircleIntersection.htm
    #
    # Calculating intersection coordinates (x1, y1) and (x2, y2) of
    # two circles of the form (x - c1.a)^2 + (y - c1.b)^2 = c1.r^2
    #                         (x - c2.a)^2 + (y - c2.b)^2 = c2.r^2
    #
    # Return value:   true if the two circles intersect
    #                 false if the two circles do not intersect
    #
      
    my_name = 'TwoBeaconIntersection()'

    # set up the bcn numbers, always lowest beacon_no in a
    i.bcn_a = min(c1.bcn_no, c2.bcn_no)
    i.bcn_b = max(c1.bcn_no, c2.bcn_no)

    # Calculating distance between circles centers
    D = math.sqrt((c1.a - c2.a) * (c1.a - c2.a) + (c1.b - c2.b) * (c1.b - c2.b))
    
    if ((c1.r + c2.r) >= D) & (D >= math.fabs(c1.r - c2.r)):
        # Two circles intersects or tangent
        # Area according to Heron's formula
        #----------------------------------
        a1 = D + c1.r + c2.r
        a2 = D + c1.r - c2.r
        a3 = D - c1.r + c2.r
        a4 = -D + c1.r + c2.r
        area = math.sqrt(a1 * a2 * a3 * a4) / 4.0
        
        # Calculating x axis intersection values
        #---------------------------------------
        val1 = (c1.a + c2.a) / 2.0 + (c2.a - c1.a) * (c1.r * c1.r - c2.r * c2.r) / (2.0 * D * D)
        val2 = 2.0 * (c1.b - c2.b) * area / (D * D)
        i.is1.x = val1 + val2
        i.is2.x = val1 - val2
        
        # Calculating y axis intersection values
        #---------------------------------------
        val1 = (c1.b + c2.b) / 2.0 + (c2.b - c1.b) * (c1.r * c1.r - c2.r * c2.r) / (2.0 * D * D)
        val2 = 2.0 * (c1.a - c2.a) * area / (D * D)
        i.is1.y = val1 - val2
        i.is2.y = val1 + val2
        
        # Intersection points are (x1, y1) and (x2, y2)
        # Because for every x we have two values of y, and the same thing for y,
        # we have to verify that the intersection points as chose are on the
        # circle otherwise we have to swap between the points
        test = math.fabs((i.is1.x - c1.a) * (i.is1.x - c1.a) + (i.is1.y - c1.b) * (i.is1.y - c1.b) - c1.r * c1.r)
        if (test > 0.0000001):
            # point is not on the circle, swap between y1 and y2
            # the value of 0.0000001 is arbitrary chose, smaller values are also OK
            # do not use the value 0 because of computer rounding problems
            tmp = i.is1.y
            i.is1.y = i.is2.y
            i.is2.y = tmp

        return (True)

    else:
        # circles are not intersecting each other
        return (False)




# function to calculate how close a point lies  to a beacon's measured radius 
def DistanceToBeaconRadius(bcn : Beacon, p: Point) -> float:

    my_name = 'DistanceToBeaconRadius()'
    

    point_distance_squared = (p.x - bcn.a)**2 +(p.y - bcn.b)**2
    beacon_radius_squared = bcn.r**2

    # check how close coordinate x, y is to the beacon radius
    distance = math.sqrt(math.fabs(point_distance_squared - beacon_radius_squared))

    print (f'{my_name} bcn = {bcn} p = {p} distance = {distance}')

    return (distance)




# of a set of 3 beacon numbers given 2 beacon numbers, return the third beacon number 
def ThirdBeacon(bcn_no_1 : int, bcn_no_2 : int, bcn_no_a : int, bcn_no_b: int, bcn_no_c: int) -> int:

    third : int         # the third beacon in the set of 3 beacons

    beacons = [bcn_no_a, bcn_no_b, bcn_no_c]
    beacons.remove(bcn_no_1)
    beacons.remove(bcn_no_2)
    third = beacons[0]

    return(third)  
    

# from the beacons found, return the beacon sets that can be trilaterated
def BcnTriLaterate (BeaconsDetected : List[int], Bcn : dict) -> List[TriLateration]:

    my_name = 'BcnTriLaterate()'

    # check which beacons intersect with each other
    bcn_intersections : List[BeaconIntersections] = []
    for index, bd in enumerate(BeaconsDetected):

        # determine the next beacon to check intersection with
        index_2 = index + 1
        while index_2 < len(BeaconsDetected):
            #print (f'{my_name} BeaconsDetected[index] = {BeaconsDetected[index]} BeaconsDetected[index_2] = {BeaconsDetected[index_2]}')

            # intersect? 
            isct = BeaconIntersections (0, 0, Point(0.0, 0.0), Point(0.0, 0.0))
            b1 = Bcn[BeaconsDetected[index]]
            b2 = Bcn[BeaconsDetected[index_2]]
            if TwoBeaconIntersection(b1, b2, isct):

                # yes, add to the list of beacon intersections
                bcn_intersections.append(isct)

            # point to the next detected beacon
            index_2 += 1
    
    #print (f'{my_name} all beacon intersections = {bcn_intersections}')

    # now look for intersections with a third beacon: b1-b2, b2-b?, B?-b1 
    # note that 'bcn_intersections' always has the lower beacon number in bcn_a, higher in bcn_b
    tri_lats : List[TriLateration] = []
    for isct_1 in bcn_intersections:
        #print (f'\n{my_name} isct_1 = {isct_1}')
        

        # look now for intersections of the second beacon  
        for isct_2 in bcn_intersections:
            if isct_1.bcn_b == isct_2.bcn_a:
                #print (f'{my_name} isct_2 = {isct_2}')

                # now look for the intersection between second and third beacon
                for isct_3 in bcn_intersections:
                    if isct_1.bcn_a == isct_3.bcn_a and isct_2.bcn_b == isct_3.bcn_b:
                        #print (f'{my_name} isct_3 = {isct_3}')

                        # create the TriLateration instance 
                        tl = TriLateration(isct_1.bcn_a, isct_1.bcn_b, isct_3.bcn_b, Point(0.0, 0.0), Point(0.0, 0.0), Point(0.0, 0.0), 0.0, Point(0.0, 0.0))
                        
                        # isct_1: determine which of the 2 intersections are closest the third beacon
                        other_bcn = ThirdBeacon(isct_1.bcn_a, isct_1.bcn_b, tl.b1, tl.b2, tl.b3)
                        print (f'{my_name} isct_1.bcn_a = {isct_1.bcn_a}, isct_1.bcn_b = {isct_1.bcn_b} other_bcn = {other_bcn}')
                        dist_1 = DistanceToBeaconRadius(Bcn[other_bcn], isct_1.is1)
                        dist_2 = DistanceToBeaconRadius(Bcn[other_bcn], isct_1.is2)
                        if dist_1 < dist_2:
                            tl.isp1.x = isct_1.is1.x
                            tl.isp1.y = isct_1.is1.y
                        else:
                            tl.isp1.x = isct_1.is2.x
                            tl.isp1.y = isct_1.is2.y

                        # isct_2: determine which of the 2 intersections are closest to the third beacon
                        other_bcn = ThirdBeacon(isct_2.bcn_a, isct_2.bcn_b, tl.b1, tl.b2, tl.b3)
                        print (f'{my_name} isct_2.bcn_a = {isct_2.bcn_a}, isct_2.bcn_b = {isct_2.bcn_b} other_bcn = {other_bcn}')
                        dist_1 = DistanceToBeaconRadius(Bcn[other_bcn], isct_2.is1)
                        dist_2 = DistanceToBeaconRadius(Bcn[other_bcn], isct_2.is2)
                        if dist_1 < dist_2:
                            tl.isp2.x = isct_2.is1.x
                            tl.isp2.y = isct_2.is1.y
                        else:
                            tl.isp2.x = isct_2.is2.x
                            tl.isp2.y = isct_2.is2.y
                        
                        # isct_3: determine which of the 2 intersections are closest to the third beacon
                        other_bcn = ThirdBeacon(isct_3.bcn_a, isct_3.bcn_b, tl.b1, tl.b2, tl.b3)
                        print (f'{my_name} isct_3.bcn_a = {isct_3.bcn_a}, isct_3.bcn_b = {isct_3.bcn_b} other_bcn = {other_bcn}')
                        dist_1 = DistanceToBeaconRadius(Bcn[other_bcn], isct_3.is1)
                        dist_2 = DistanceToBeaconRadius(Bcn[other_bcn], isct_3.is2)
                        if dist_1 < dist_2:
                            tl.isp3.x = isct_3.is1.x
                            tl.isp3.y = isct_3.is1.y
                        else:
                            tl.isp3.x = isct_3.is2.x
                            tl.isp3.y = isct_3.is2.y

                        # calculate a linear approximation of the circumferance of the 3 point intersection
                        side_a = math.sqrt((tl.isp1.x-tl.isp2.x)**2 + (tl.isp1.y-tl.isp2.y)**2)
                        side_b = math.sqrt((tl.isp2.x-tl.isp3.x)**2 + (tl.isp2.y-tl.isp3.y)**2)
                        side_c = math.sqrt((tl.isp1.x-tl.isp3.x)**2 + (tl.isp1.y-tl.isp3.y)**2)
                        tl.d123 = side_a + side_b + side_c

                        # calculate the centroid point (center of gravity)
                        tl.centroid.x = (tl.isp1.x + tl.isp2.x + tl.isp3.x)/3
                        tl.centroid.y = (tl.isp1.y + tl.isp2.y + tl.isp3.y)/3  

                        tri_lats.append(tl)
                        break      
    
    return(tri_lats)





# are 2 numbers equal within given delta?
# warning: floating point calculations are not exact, i.e. 2.5 - 2.6 = 0.10000000000000009
def equal_delta(a: float, b: float , delta: float) -> bool:

    return np.abs(a - b) <= delta   


def show_camera_settings(hhmmss: str):
    
    my_name = 'show_camera_settings()'
    filename = gv.ram_disk + hhmmss + '_settings' + '.txt'
    try:
        with open(filename, 'w') as f:
                
            f.write (f'camera.resolution = {camera.resolution}\n')
            
            f.write (f'camera.exposure_mode = {camera.exposure_mode}\n')
            f.write (f'camera.meter_mode = {camera.meter_mode}\n')
            f.write (f'camera.awb_mode = {camera.awb_mode}\n')
            f.write (f'camera.sensor_mode = {camera.sensor_mode}\n\n')
         
            f.write (f'camera.iso = {camera.iso}\n')
            f.write (f'camera.exposure_speed = {camera.exposure_speed}\n')
            f.write (f'camera.shutter_speed = {camera.shutter_speed}\n')
            f.write (f'camera.framerate = {camera.framerate}\n\n')
           
            f.write (f'camera.analog_gain = {camera.analog_gain}\n')
            f.write (f'camera.digital_gain = {camera.digital_gain}\n')
            f.write (f'camera.awb_gains = {camera.awb_gains}\n\n')
        
            f.write (f'camera.brightness = {camera.brightness}\n')
            f.write (f'camera.contrast = {camera.contrast}\n')
            f.write (f'camera.saturation = {camera.saturation}\n')
            f.write (f'camera.sharpness = {camera.sharpness}\n\n')
            
            f.write (f'camera.drc_strength = {camera.drc_strength}\n')
            f.write (f'camera.exposure_compensation = {camera.exposure_compensation}\n')
            f.write (f'camera.framerate_range = {camera.framerate_range}\n')
            f.write (f'camera.image_effect = {camera.image_effect}\n')
            f.close()
    except:
        print (f'{my_name} Could not create file {filename}')
    
    return()
    


    
# rotate the camera by the requested number of steps (positive or negative) 
def rotate_cam(no_steps: int, direction: bool):

    my_name = 'rotate_cam()'

    # as the mirror motor is mounted upside down, invert the rotation direction
    rot_direction = not direction
    
    # delay in seconds for 200 step stepping motor, any faster will loose steps
    delay = 0.005

    if my_gpio.Im_a_Raspberry:
        
        # set the direction
        GPIO.output(my_gpio.cr_DIR_PIN, rot_direction)
        
        # and rotate for the requested number of steps
        for x in range(no_steps):
            GPIO.output(my_gpio.cr_STEP_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(my_gpio.cr_STEP_PIN, GPIO.LOW)
            time.sleep(delay)
    
    return()



# rotate the mirror until the calibration point is reached 
def calibrate_mirror() -> bool:

    my_name = 'calibrate_mirror()'
    
    calibrated = False
    direction  = my_gpio.CW
    step_incr  = 1
    step_count = 0
        
    if my_gpio.Im_a_Raspberry:
        
        # set the direction
        GPIO.output(my_gpio.cr_DIR_PIN, my_gpio.CCW)
        
        # and rotate until the calibration point is found or a full rotation has completed
        while not (calibrated or step_count > my_gpio.cr_SPR):
            
            rotate_cam(step_incr, direction)
            step_count = step_count + step_incr
            #print (f'{my_name}: step_count = {step_count}')
            
            # and check if the calibration point has been found
            calibrated = not GPIO.input(my_gpio.mirror_prox_pin)
        
    return(calibrated)


    
        
    
# calculate the intersection of 2 linear equations
def intersect(m1: float, b1: float, m2: float, b2: float) ->Tuple[float, float]:

    my_name = 'intersect()'

    # m1 * x + b1 = m2 * x + b2
    # (m1 - m2)* x = b2 - b1
    # x = (b2 - b1) / m1 - m2)
    x = (b2 -b1) / (m1 - m2)
    
    # and from the y coordinate, calculate the y coordinate from one of the two lines
    y = m1 * x + b1
    print (f'{my_name}: x = {x} y =  {y}')
    return (x,y)


                

# get the compass bearing from the QMC5883L compass
def get_compass_bearing() -> float:
    
    my_name = 'get_compass_bearing()'

    # returns the True compass bearing, compansated by magnetic declination 
    # magnetic decination in Glastone QLD is 12.2 degrees East
    magn_decl = 12.2
    
    if compass_present:
        b0 = sensor.get_bearing()
        time.sleep(0.1)
        b1 = sensor.get_bearing()
        time.sleep(0.1)
        b2 = sensor.get_bearing()
        
        # calculate the average bearing
        b = brng_avg(b0, b1, b2)
        
        # north and south are flipped, also magnetic declination, correct
        b = brng_add(b, 180.0 + magn_decl)
        
        # round, 1 decimals
        b = round(b, 1)
    else:
        b = 0.0
        
    return (b)



# add two bearings
def brng_add(bearing_1: float, bearing_2: float) -> float:
    sum = (bearing_1 + bearing_2) % 360.0
    return (sum)
    

# subtract two bearings
def brng_diff(bearing_1: float, bearing_2: float) -> float:

    r = (bearing_2 - bearing_1) % 360.0
    # Python modulus has same sign as divisor, which is positive here,
    # so no need to consider negative case
    if r >= 180.0:
        r -= 360.0
    return (r)



# calculate the average of 3 bearings
def brng_avg(b0: float, b1: float , b2: float) -> float:

    # around the 360 mark, it is tricky. This give ok results over the full range
    b = (brng_diff(360.0,b0) + brng_diff(360.0,b1) + brng_diff(360.0,b2)) /3.0
    b = brng_add(360.0, b)
    return(b)
 
 
    
# wait until the drive or zero turn command is completed		
def wait_cmd_done():

    # need to sleep for a brief moment to allow the arduino driver to process the cmd
    time.sleep(0.5)
    
    # wait until status indicates completed
    cmd_finished = GPIO.input(my_gpio.cr_DRV_PIN)
    while not cmd_finished:
        #print (f'{my_name} loop cmd_finished = {cmd_finished}')
        time.sleep(0.5)
        cmd_finished = GPIO.input(my_gpio.cr_DRV_PIN)

    return



# turn to the requested bearing and drive
def turn_drive(req_bearing: float, dist_cm: float, mode: int) -> float:

    # mode - manual_mode 
    #      - auto_mode

    my_name = 'turn_drive()'

    # motor reduction = 60:1, sprockets 1:1, wheel diameter 25 cm
    # 1 step (revolution) = 78.53/60 = 1.0309 cm
    steps_cm   = 0.764         # wheel: steps per cm
    
    # get the absolute bearing the mower is pointing to
    current_bearing = get_compass_bearing()
    print (f'{my_name}: current_bearing = {current_bearing} req_bearing {req_bearing}')
    
    # in manual mode, the mower is operated from the hmi directly and req_bearing is the requested turn angle
    # either positive or negative (counter clockwise)
    if mode == gnl.manual_mode:
        # calculate the requested bearing
        req_bearing = brng_add(current_bearing,req_bearing)
       
    # correct if necessary to make sure we are going in the right direction; with
    # a turning circumferance of 153.9cm, 1 degree will be 0.427 cm.  1 step will be 2.3 degrees. 
    # do not correct any less than 3 degrees. 
    cnt = 0
    while not equal_delta(current_bearing, req_bearing, 3.0):
    
        # only try a few times
        cnt += 1
        
        # calculate the angle to turn
        turn_angle = brng_diff(current_bearing, req_bearing)
        print (f'{my_name}: cnt = {cnt} current_bearing {current_bearing} req_bearing {req_bearing} turn_angle = {turn_angle}')
        
        # calculate the distance the wheels need to travel
        turn_cm = turn_angle  / 360.0 * math.pi * gv.mower_width
        turn_steps = int(turn_cm * steps_cm)
        
        # zero turn at 50% duty
        if my_gpio.Im_a_Raspberry:
            mower_cmd('z,' + str(turn_steps) + ',0,50')
            wait_cmd_done()
    
        # check if we are close
        current_bearing = get_compass_bearing()
    
        if cnt > 10:
            print (f'{my_name}: current_bearing = {current_bearing} unable to reach req_bearing = {req_bearing}')
            exit()
            break
    
    # calculate the steps to drive
    drive_steps = int(dist_cm * steps_cm)

    # drive at 80% duty    
    if my_gpio.Im_a_Raspberry:
        mower_cmd('d,' + str(drive_steps) + ',0,80')
        wait_cmd_done()
           
    return (current_bearing)
 


 
    

# send command to Arduino based motor driver
def mower_cmd(command: str) -> bytes:

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

    my_name = 'mower_cmd()'

    # Arduino Motor driver registers
    reg_cmd       = 0            # 1 bytes, used by drive command
    reg_drive     = 1            # 6 bytes, used by drive command
    reg_tune      = 7            # 12 bytes, used by tune command
    reg_motor     = 19           # 4 bytes, used by show command
    reg_next      = 23           # next available register

    data = ''
    
    print (f'{my_name} command = {command}')


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
                print (f'{my_name} b_reg_drive')

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
                print (f'{my_name} b_reg_drive')
                
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
        print (f'{my_name} cmd data = {data}')
        

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
                print (f'{my_name} b_reg_tune = {b_reg_tune}')

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
        print (f'{my_name} tuning data =  {data}')
        
        # let i2c bus recover
        time.sleep(0.1)
        
        # read 2 x 2 bytes containing the motor counters
        l_reg_motor = bus.read_i2c_block_data(ARD_ADDR, reg_motor, 4)
        
        # convert from list to bytearray
        b_reg_motor = bytearray(l_reg_motor)
        data = struct.unpack('<hh', b_reg_motor)    # short integer
        print (f'{my_name} motor_data =  {data}')
                
                
    # real all registers command: r
    elif command[:1] == 'r':                    # slice first character of command
    
        # read the entire register back from the Arduino
        data = bus.read_i2c_block_data(ARD_ADDR, reg_cmd, reg_next)
        print (f'{my_name} all registers =  {data}')
         
    # quit
    elif command == 'q':
        exit()
	
    return(data)



# the work area is given in coordinates in cm. Now calculate the coordinates that
# the centre mower should adhere to due to it's width and the linear equations of each of the segments 
# of the work area
def calc_work_path(work_area_coord_cm: List) -> List:

    my_name = 'calc_work_path()'

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
            (m, b_in, mid_coord) = get_lin_eq_segment(last_coord, coord, gv.mower_width/2.0)
            
            # store only the inner boundary and the mid coordinate, which we  need only in case of a vertical segment
            segments_0.append((m, b_in, mid_coord))
        else:
            # first coordinate, store it
            first_coord = coord
            
        # save coord for the next segment
        last_coord = coord
        
    # now create the last segment by connecting to the first coordinates
    (m, b_in, mid_coord) = get_lin_eq_segment(last_coord, first_coord, gv.mower_width/2.0)
    
    # store only the inner boundary
    segments_0.append((m, b_in, mid_coord))        
     
    #print (f'{my_name} segments_0 = {segments_0}')
    
    
    
    
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
    
    #print (f'{my_name} segments_1 = {segments_1}')
   
   
   
   
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
        (abs_brng, trvl_dist_cm) = calc_course(xs, ys, xe, ye)
        
        # now build the final work area data list
        work_path.append((xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm))
    
    #print (f'{my_name}: work_path = {work_path}')
    return (work_path)

    
    

# determine linear equation y = mx + b of line through 2 coordinates (x1, y1) and (x2, y2). Then return 
# the linear equation of the innner boundary. This basically the same line, but offset with the mower's width.
# this funtion assumes a clockwise direction of the mower to determine the offset. Note that the mower
# width can be set to 0. Then also return the coordinate of the mid point.
def get_lin_eq_segment (x1y1, x2y2, offset):

    my_name = 'get_lin_eq_segment()'

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

    my_name = 'calc_segment_cross()'

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



# get the absolute bearing starting at coordinate (xs,ys) and ending at (xe,ye) and the distance between
def calc_course(xs, ys, xe, ye) -> Tuple[float, float]:

    my_name = 'calc_course()'

    # initialize as floats
    brg = -1.0
    distance = -1.0
    
    # calculate the deltas
    dx = xe - xs
    dy = ye - ys
    #print (f'{my_name} dx = {dx} dy = {dy}')
    
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
def in_range(val: float, lim1: float, lim2: float) -> bool:

    if lim1 > lim2:
        tmp = lim2
        lim2 = lim1
        lim1 = tmp
        
    inrange: bool = (val >= lim1) and (val <= lim2)
    
    return (inrange)




def traditional_pattern(work_area_limits, res_q):

    my_name = 'traditional_pattern()'

    print (f'{my_name}: not implemented yet')
    
    return()
    


def spiral_pattern(work_area_limits, res_q):
    
    my_name = 'spiral_pattern()'
    
    global veh_pos_cm


    
    circumf = 0
    last_circumf = float('inf')          # initialize at largest possible int
    total_distance = 0.0                 # distance traveled
    
    # announce spiral pattern has started
    msg = "Spiral pattern started at " + str(datetime.datetime.now())
    que.push_queue(res_q, que.t_message, msg)

    # set up the beacon data dictionary from global variable beacon positions collected in the main
    bcn : Beacon
    bcn_data = {}
    for bp in gv.beacon_pos:
        #unpack
        b_no, b_x, b_y = bp

        # build the dictionary
        bcn_data[b_no] = Beacon (bcn_no = b_no,  a = b_x,   b = b_y, r = 0.0)
    print (f'{my_name}: bcn_data = {bcn_data}')


    # get the vehicle position, note that the vehicle bearing is not known yet
    (xs,ys,veh_brng) = gv.veh_pos_cm
    
    # get the current vehicle bearing
    veh_brng = get_compass_bearing()
    print (f'{my_name}: veh_brng {veh_brng}')
    
    # send the updated vehicle position to the hmi
    gv.veh_pos_cm = (xs,ys,veh_brng)
    que.push_queue(res_q, que.t_vehicle_position, gv.veh_pos_cm)
    
    print (f'{my_name}: work_area_limits = {work_area_limits}')
    print (f'{my_name}: gv.veh_pos_cm = {gv.veh_pos_cm}')
    
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
            print (f'\n\n{my_name}: seg {seg}')           
                
            # prepare the path for the next interation
            new_path.append((xs,ys))

            # get the current vehicle position
            (x_veh, y_veh, veh_brng) = gv.veh_pos_cm
            print (f'{my_name}: This should be my position - gv.veh_pos_cm = {gv.veh_pos_cm}')
            
            # check the current position by triangulation
            position_found, tri_lat = get_veh_position(bcn_data)
            
            # update the hmi if no position found
            if position_found: 
                (x_veh, y_veh) = tri_lat.centroid.x, tri_lat.centroid.y 
                veh_pos_cm_upd = (x_veh, y_veh, veh_brng)
                print (f'{my_name}: Position calculated by triangulation: veh_pos_cm_upd {veh_pos_cm_upd}')
                msg = "Position calculated by triangulation: " + str(x_veh) + ' ' + str(y_veh) + ' bearing ' + str(veh_brng) + ' ' + str(datetime.datetime.now())
                que.push_queue(res_q, que.t_message, msg)
            else:
                print (f'{my_name}: No beacons detected')
                msg = "No beacons detected " + str(datetime.datetime.now())
                que.push_queue(res_q, que.t_message, msg)
            
            # check if we need to drive to the start postion
            if (x_veh != xs) or (y_veh != ys):

                # get bearing and distance to (xs, ys) from where we are
                (start_brng, start_dist_cm) = calc_course(x_veh, y_veh, xs, ys)
                
                # zero turn and drive to the start position
                print (f'{my_name}: Moving to start position - bearing {start_brng} distance {start_dist_cm} --- {x_veh} {y_veh} to {xs} {ys}')
                veh_brng = turn_drive(start_brng, start_dist_cm, gnl.auto_mode)
                total_distance += start_dist_cm
                
            # send the vehicle position to the hmi 
            gv.veh_pos_cm = (xs,ys,veh_brng)
            que.push_queue(res_q, que.t_vehicle_position, gv.veh_pos_cm)
            print (f'{my_name}: now at position {gv.veh_pos_cm}')
                
                
            # zero turn to the required bearing to xe, ye
            print (f'{my_name}: zero turning to abs bearing {abs_brng}')

            # mow the segment (xs,ys) to (xe,ye) 
            print (f'{my_name}: Now mowing to {xe}, {ye}')
            veh_brng = turn_drive(abs_brng, trvl_dist_cm, gnl.auto_mode)
            total_distance += trvl_dist_cm
            
            # send the vehicle position to the hmi 
            gv.veh_pos_cm = (xe,ye, veh_brng)
            que.push_queue(res_q, que.t_vehicle_position, gv.veh_pos_cm)
            
            print (f'{my_name}: now at position {gv.veh_pos_cm}')
            
        # save the circumference
        last_circumf = circumf
        
        # calculate the new work area limits
        #print (f'{my_name} new_path = {new_path}')
        work_area_limits = []
        work_area_limits = calc_work_path(new_path)   
        
        # calculate the circumference of the new work area
        circumf = 0
        for seg in work_area_limits:
            (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm) = seg
            circumf = circumf + trvl_dist_cm 
        
        # calculate the change in circumference
        delta_circumf = last_circumf - circumf
        print (f'{my_name}: circumf:  {circumf}  {delta_circumf}')
 
    # announce spiral pattern has completed
    msg = "Spiral pattern completed at " + str(datetime.datetime.now())
    que.push_queue(res_q, que.t_message, msg)
    msg = "Total distance: " + str(total_distance)
    que.push_queue(res_q, que.t_message, msg)
                   
    return()
    
     
    
def random_pattern(work_area_limits, res_q):

    my_name = 'random_pattern()'

    print (f'{my_name}: not implemented yet')
    
    return()


 




# the main for thread mowing
def mowing(cmd_q, res_q):
    
    global veh_pos_cm                   # the vehicle position in cm
    global veh_pos_tri_cm               # the vehicle position as determined by triangulation
    global mowing_pattern               # the mowing pattern requested
    global work_area_coord_cm           # the work area coordinates in cm
    global beacon_pos_cm                # the beacon positions    
    global bus                          # I2C bus
    global signal_mowing                # signal to stop mowing pattern
    
    my_name = 'mowing()'
    
    work_area_limits = []       # list of work area limits per segment;
                                # (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm)

    
    # rotate the mirror clock-wise until the calibration point is found
    calibrated = calibrate_mirror()
    if not calibrated:
        print (f'{my_name}: mirror calibration point not found after 1 full rotation')
        GPIO.cleanup()
        exit(0)
    else:
        
        # calibration point was found, now rotate the mirror to face front
        # the number of steps to rotate is determined by the physical contruction
        rotate_cam(200, my_gpio.CCW)                
        print (f'{my_name}: zero point reached')
        
            
    while True:
                               
        # get any commands from the hmi
        pkgs = que.get_queue(cmd_q)
        print (f'{my_name} mower thread pkgs received= {pkgs} gv.kill_thread_1 = {gv.kill_thread_1}')

        for pkg in pkgs:
        
            print (f'{my_name}: pkg received {pkg}')
            
            (pkg_time, ptype, data) = pkg
            now = time.time()
            if now - pkg_time < que.expire_threshold:
        
                # start mowing pattern command from hmi
                if ptype == que.t_start_mowing:

                    # unpack the start vehicle position, mow pattern, the work area and beacon positions
                    (gv.veh_pos_cm, mowing_pattern, work_area_coord_cm, gv.beacon_pos_cm) = data
                    print (f'{my_name} gv.veh_pos_cm = {gv.veh_pos_cm} mowing_pattern = {mowing_pattern} gv.beacon_pos_cm = {gv.beacon_pos_cm}')
                 
                    
                    # from the given work area, calculate the coordinates and equations that the mower should adhere to
                    work_area_limits = calc_work_path(work_area_coord_cm)

                    # signal to allow the mowing pattern, this global variable is checked by the relevant routines 
                    stop_mowing = False
                     
                    # and start the pattern, pass the response queue to allow dat ato be posted back to the hmi
                    if mowing_pattern == gv.mow_traditional:             
                        traditional_pattern(work_area_limits, res_q)
                    else:
                        if mowing_pattern == gv.mow_spiral:
                            spiral_pattern(work_area_limits, res_q)
                        else:
                            if mowing_pattern == gv.mow_random:
                                random_pattern(work_area_limits, res_q)
                     

                                
                
                # stop mowing pattern command from hmi, this package type has no data
                if ptype == que.t_stop_mowing:

                    # signal to stop the mowing pattern immediately, this global variable is monitored by the relevant functions
                    stop_mowing = True
                    
                        
                # manual command
                if ptype == que.t_man_mv_forwrd or ptype == que.t_man_mv_cntclk or \
                   ptype == que.t_man_mv_clk    or ptype == que.t_man_mv_bckwrd:    
                       
                    (turn_angle,dist_cm) = data
                    print (f'{my_name} manual cmd - req turn_angle = {turn_angle} req distance = {dist_cm}')
                    turn_drive(turn_angle, dist_cm, gnl.manual_mode)                              
                
                                        
                # manual Stop command
                if ptype == que.t_man_mv_stop:
                    (turn_angle,dist_cm) = data
                    print (f'{my_name} Stop cmd')
            
            
        # check if this thread needs to exit
        if gv.kill_thread_1: 
            
            #  release I2C bus
            bus.close()
            bus = None      
            return()
        
        # sleep for 1 second
        time.sleep(1)
            
                    
    return()
