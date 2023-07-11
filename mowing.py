#!/usr/bin/python3

import sys
import math
import time
import struct
import datetime             # used by spiral_pattern

import numpy as np
import cv2
from picamera2 import Picamera2
import apriltag
import RPi.GPIO as GPIO
import smbus                # I2C related
import py_qmc5883l          # QMC5883 compass

# application imports
import global_definitions as gd
import my_queue as que
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

# global variables in mowing.py

mowing_debug_log = './mowing_dbug.log'  # debug log
ram_disk = '/mnt/ramdisk/'      # use a ram disk to save the SD card
    
mower_width = 49                # mower width in cm
beacon_pos_cm = []              # beacon positions in cm
compasses = []                  # compass instances
veh_pos_cm = (0,0,-1)           # vehicle position (x,y) in cm and vehicle bearing

# drive mode
auto_mode = 0
manual_mode = 1

# camera
frm_dim = {'x':4056,'y':3040}   # frame dimension

# enable the blade
blade_enabled = False

# acceptable location accuracy
accuracy_cm = 10.0

# return the distance to the april tag based upon the measured vertical rib of the tag
def april_distance(v_rib: float) -> float:

    # based upon 17.7 cm vertical rib apriltag and 3040 resolution
    # the distance calculated from the vertical rib (in pixels) of the April tag is not linear.
    # Calculated by Excel regression: dist = (v_rib ^ -1.005) * 93343
    A = 93343.0
    B = -1.005
    dist = (v_rib ** B) * A

    return (dist)
    




# get the vehicle position 
def get_veh_position(camera, bcn_data : dict) -> Tuple[bool, TriLateration]:

    trilat = TriLateration(0, 0, 0, Point(0.0, 0.0), Point(0.0, 0.0), Point(0.0, 0.0), 0.0, Point(0.0, 0.0))
    best_trilat = TriLateration(0, 0, 0, Point(0.0, 0.0), Point(0.0, 0.0), Point(0.0, 0.0), 0.0, Point(0.0, 0.0))
                                            
    my_name = 'get_veh_position()'

    view_angle = 22.5             # the camera view angle in degrees for the 8mm lens, 16 images/rotation
    
    # initialize
    position_found = False
    veh_pos_cm_tri = Point(0,0)
    found_bcn_list = []     # found beacon list for this request
    
    # rotate the mirror in whole steps for 1 full rotation
    step_incr = int(view_angle / 360.0 * my_gpio.MIR_SPR)          # keep as an integer
    no_steps = 0
    rel_mirror_brng = 0 
    while(no_steps < my_gpio.MIR_SPR):
    
        # sleep to stabilize from vibrations
        time.sleep(0.2)
        
        # capture single frame directy into opencv object
        frame = camera.capture_array("main")
        
        # save the settings
        hhmmss = datetime.datetime.now().strftime("%I%M%S")
        
        # for debugging
        show_camera_settings(camera, hhmmss)
        
        # crop the image to a square, saves time
        y1 = 0
        y2 = frm_dim['y']
        x1 = int((frm_dim['x']-frm_dim['y'])/2)
        x2 = x1 + frm_dim['y']
        frame_cropped = frame[y1:y2, x1:x2]
        
        # grab the dimensions of the image and calculate the center of the frame
        (h, w) = frame_cropped.shape[:2]
        (cX, cY) = (w // 2, h // 2)
        
        # the frame is rotated due to the mirror rotation, de-rotate it to face to the front
        camera_orientation = 0             # camera points North in reality, no correction required
        corr_angle =  brng_add(rel_mirror_brng, camera_orientation) * -1
        #dbg.write(f'{my_name}: corr_angle = {corr_angle:.2f}\n')
        M = cv2.getRotationMatrix2D((cX, cY), corr_angle, 1.0)
        frm_rotated = cv2.warpAffine(frame_cropped, M, (w, h))
        
        # flip the image around the y-axis due to the mirror effect
        frm_flipped = cv2.flip(frm_rotated, 1)
        
        # save the rotated color image for evaluation
        fn = ram_disk + hhmmss + 'img_rot_' + str(rel_mirror_brng) +'.jpg'
        cv2.imwrite(fn, frm_flipped) 
        
        # use a filter to blur the image and get rid of noise
        # the filter size must be odd and a function of the resolution
        frm_blur = cv2.GaussianBlur(frm_flipped, (9,9), 0)

        # convert to HSV colors for easy color detection
        frm_hsv =cv2.cvtColor(frm_blur, cv2.COLOR_BGR2HSV)

        # tuned color for cyan which is not abundant in nature
        # HSV - Hue Saturation Value 0-180, 0-255, 0-255.
        #lower_col = np.array([90, 50, 0])       # 90, 50, 0
        #upper_col = np.array([110,255,255])
        
        # To reduce low light scatter and accept slight lighter shade of cyan
        lower_col = np.array([85, 35, 75])
        upper_col = np.array([110,255,255])
        
        mask = cv2.inRange(frm_hsv, lower_col, upper_col)
        fn = ram_disk + hhmmss + 'img_msk_' + str(rel_mirror_brng) +'.jpg'
        cv2.imwrite(fn, mask)
        
        # define the AprilTags detector options and then detect the AprilTags in the input image
        options = apriltag.DetectorOptions(families="tag25h9")
        detector = apriltag.Detector(options)
        
        #   warning: too many borders in contour_detect (max of 32767!)
        #   Segmentation fault
        #
        # sometimes too many segments are detected as there are too many objects
        # in the image, opencv library issue 
        
        print(f'{my_name} before april tag detector\n')
        results = detector.detect(mask)
            
        print(f'{my_name} total AprilTags detected {(len(results))}\n')
        print(f'results = {results}\n')
        
        # there may be multiple apritags detected
        for r in results:
            
            dbg.write(f'{my_name}: tag_id = {r.tag_id}\n')
            
            # do not add duplicate tags in case of overlapping images
            if r.tag_id not in found_bcn_list:
                
                # calculate the distance of the 2 vertical ribs of the 4 corners
                (ptA, ptB, ptC, ptD) = r.corners
                x1 = ptD[0] - ptA[0]
                y1 = ptD[1] - ptA[1]
                v1 = math.sqrt(x1*x1 + y1*y1)
                
                x2 = ptC[0] - ptB[0]
                y2 = ptC[1] - ptB[1]
                v2 = math.sqrt(x2*x2 + y2*y2)
                        
                # take the largest vertical rib
                dbg.write(f'{my_name}: v1= {v1} pixels v2= {v2} pixels\n')
                v = max(v1, v2)
                
                #  calculate the distance based upon the vertical rib
                distance = april_distance(v)
                print(f'{my_name} distance = {distance:.2f} cm\n')
            
                # add 5cm to each beacon to increase the chance of a trilat
                distance += 5.0
                
                # and add the apriltag number to the beacon table
                found_bcn_list.append(r.tag_id)
                bcn_data[r.tag_id].r = distance
        
        
        # rotate the mirror
        rotate_cam(step_incr, my_gpio.CW)
        no_steps = no_steps + step_incr
                
        # calculate the angle 
        rel_mirror_brng = int(no_steps / my_gpio.MIR_SPR * 360.0)
        #dbg.write(f'{my_name}: rotating no_steps = {no_steps} rel_mirror_brng = {rel_mirror_brng:.2f}\n')
    
    # full rotation is required, correct in case it was under 
    step_makeup = my_gpio.MIR_SPR  - (my_gpio.MIR_SPR // step_incr) * step_incr # '//' is integer division
    if step_makeup > 0:
        dbg.write(f'{my_name}: making up steps for full rotation = {step_makeup}\n')
        rotate_cam(step_makeup, my_gpio.CW)
    
    # print debug data
    dbg.write(f'{my_name}: found_bcn_list = {found_bcn_list}\n')
    dbg.write(f'{my_name}: bcn_data = {bcn_data}\n')
    
    # find which beacons can be trilaterated
    trilats = BcnTriLaterate(found_bcn_list, bcn_data)

    if len(trilats) > 0:

        position_found = True
        dbg.write(f'{my_name} trilats = {trilats}\n')
    
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

    dbg.write(f'{my_name} bcn = {bcn} p = {p} distance = {distance:.2f}\n')

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
            #dbg.write(f'{my_name} BeaconsDetected[index] = {BeaconsDetected[index]} BeaconsDetected[index_2] = {BeaconsDetected[index_2]}\n')

            # intersect? 
            isct = BeaconIntersections (0, 0, Point(0.0, 0.0), Point(0.0, 0.0))
            b1 = Bcn[BeaconsDetected[index]]
            b2 = Bcn[BeaconsDetected[index_2]]
            if TwoBeaconIntersection(b1, b2, isct):

                # yes, add to the list of beacon intersections
                bcn_intersections.append(isct)

            # point to the next detected beacon
            index_2 += 1
    
    #dbg.write(f'{my_name} all beacon intersections = {bcn_intersections}\n')

    # now look for intersections with a third beacon: b1-b2, b2-b?, B?-b1 
    # note that 'bcn_intersections' always has the lower beacon number in bcn_a, higher in bcn_b
    tri_lats : List[TriLateration] = []
    for isct_1 in bcn_intersections:
        #dbg.write(f'\n{my_name} isct_1 = {isct_1}\n')
        

        # look now for intersections of the second beacon  
        for isct_2 in bcn_intersections:
            if isct_1.bcn_b == isct_2.bcn_a:
                #dbg.write(f'{my_name} isct_2 = {isct_2}\n')

                # now look for the intersection between second and third beacon
                for isct_3 in bcn_intersections:
                    if isct_1.bcn_a == isct_3.bcn_a and isct_2.bcn_b == isct_3.bcn_b:
                        #dbg.write(f'{my_name} isct_3 = {isct_3}\n')

                        # create the TriLateration instance 
                        tl = TriLateration(isct_1.bcn_a, isct_1.bcn_b, isct_3.bcn_b, Point(0.0, 0.0), Point(0.0, 0.0), Point(0.0, 0.0), 0.0, Point(0.0, 0.0))
                        
                        # isct_1: determine which of the 2 intersections are closest the third beacon
                        other_bcn = ThirdBeacon(isct_1.bcn_a, isct_1.bcn_b, tl.b1, tl.b2, tl.b3)
                        dbg.write(f'{my_name} isct_1.bcn_a = {isct_1.bcn_a}, isct_1.bcn_b = {isct_1.bcn_b} other_bcn = {other_bcn}\n')
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
                        dbg.write(f'{my_name} isct_2.bcn_a = {isct_2.bcn_a}, isct_2.bcn_b = {isct_2.bcn_b} other_bcn = {other_bcn}\n')
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
                        dbg.write(f'{my_name} isct_3.bcn_a = {isct_3.bcn_a}, isct_3.bcn_b = {isct_3.bcn_b} other_bcn = {other_bcn}\n')
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


def show_camera_settings(camera, hhmmss: str):
    

    
    my_name = 'show_camera_settings()'
    filename = ram_disk + hhmmss + '_settings' + '.txt'
    try:
        with open(filename, 'w') as f:
            
            f.write ((hhmmss + ',' + str(camera.capture_metadata()) + '\n'))    
            f.close()
    except:
        print(f'{my_name} Could not create file {filename}\n')
    
    return()



    
# rotate the camera by the requested number of steps (positive or negative) 
def rotate_cam(no_steps: int, direction: bool):

    my_name = 'rotate_cam()'
    
    # delay in seconds for 200 step stepping motor, any faster will loose steps
    delay = 0.005

    # set the direction
    GPIO.output(my_gpio.MIR_DIR_PIN, direction)
    
    # and rotate for the requested number of steps
    for x in range(no_steps):
        GPIO.output(my_gpio.MIR_STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(my_gpio.MIR_STEP_PIN, GPIO.LOW)
        time.sleep(delay)
    
    return()



# rotate the mirror until the calibration point is reached 
def calibrate_mirror() -> bool:

    my_name = 'calibrate_mirror()'
    
    calibrated = False
    direction  = my_gpio.CCW
    step_incr  = 1
    step_count = 0
        
    # set the direction
    GPIO.output(my_gpio.MIR_DIR_PIN, direction)
    
    # and rotate until the calibration point is found or a full rotation has completed
    while not (calibrated or step_count > my_gpio.MIR_SPR):
        
        rotate_cam(step_incr, direction)
        step_count = step_count + step_incr
        #dbg.write(f'{my_name}: step_count = {step_count}\n')
        
        # and check if the calibration point has been found
        calibrated = not GPIO.input(my_gpio.MIR_ZERO_PIN)
    
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
    dbg.write(f'{my_name}: x = {x:.2f} y =  {y:.2f}\n')
    return (x,y)

                

# get the compass bearing from the QMC5883L magnetic sensor (compasses)
def get_compass_bearing(magsensors) -> float:
    
    my_name = 'get_compass_bearing()'

    bearings = []
    
    # returns the True compass bearing, compansated by magnetic declination 
    # magnetic decination in Gladstone QLD is 9.5 degrees East
    # magn_decl = 9.5
    
    # get the bearings of the 3 compasses
    bearings = get_bearings(bus, my_gpio.MUX_BUSES, magsensors) 
    dbg.write(f'{my_name} bearings = {bearings}\n')
    
    # calculate the average bearing
    b_avg = calc_avg_bearing(bearings)
    dbg.write(f'{my_name} b_avg = {b_avg:.2f}\n')
    
    # adjust for the non-linearities of the compass
    corr_bearing:float = correct_bearing(b_avg)
    dbg.write(f'{my_name} corr_bearing = {corr_bearing:.2f}\n')
    
    # should never happen, so in that case, fatal error
    if corr_bearing == -1:
        exit
        
    # correct for how the compass is mounted on the mower
    corr_bearing = brng_add(corr_bearing, 90.0)
    dbg.write (f"corr_bearing {corr_bearing}\n")
        
    return (corr_bearing)


def correct_bearing(uncorr_brng: float) -> float:

    # Returns a corrected bearing for a specific compass and calibration
    #
    #   uncorr_brng   float   Uncorrected bearing in degrees [-9.830, 365.375]
    #   corr_brng     float   Returns a corrected bearing in degrees [0.0-360.0]
    #                         In case of error, returns the uncorrected bearing

    # the magnetic compass bearing is compensated for mounting on the mower,
    # however non-linearities affect accuracy. Once the compasses are 
    # recalibrated, the effective bearing of the average compass bearing must 
    # be determined. This was done by rotating the mower by 10 degrees and 
    # recording the effective bearing. This resulted in array brng_corr_table.
    
    # bearing correction table, i.e. magnetic compass bearing 15.375 degrees 
    # is actual bearing 20 degrees magnetic (no declination correction).
    # Based upon actual measurements (15/3/2023). Note that the result still needs
    # to be corrected for the mounting of the compass in the mower.
    
    #                        uncorr_brng,actual_brng
    brng_corr_table = np.array([    [-9.830,0],
                                    [5.375,10],
                                    [15.038,20],
                                    [25.263,30],
                                    [36.064,40],
                                    [46.106,50],
                                    [57.621,60],
                                    [68.353,70],
                                    [80.161,80],
                                    [90.572,90],
                                    [102.890,100],
                                    [113.569,110],
                                    [124.888,120],
                                    [136.268,130],
                                    [146.971,140],
                                    [157.317,150],
                                    [166.562,160],
                                    [175.669,170],
                                    [185.813,180],
                                    [194.776,190],
                                    [204.643,200],
                                    [212.349,210],
                                    [221.049,220],
                                    [230.216,230],
                                    [239.786,240],
                                    [248.568,250],
                                    [258.067,260],
                                    [268.250,270],
                                    [275.652,280],
                                    [285.283,290],
                                    [294.839,300],
                                    [305.008,310],
                                    [314.615,320],
                                    [322.122,330],
                                    [331.473,340],
                                    [340.423,350],
                                    [350.170,360],
                                    [365.375,370]])

    # initialize
    (no_elements, tt ) = brng_corr_table.shape
    
    # check if the uncorrected bearing is within the range of the brng_corr_table
    (brng_l, corr_brng_l) = brng_corr_table[0]
    (brng_h, corr_brng_h) = brng_corr_table[no_elements - 1] 
    if uncorr_brng >= brng_l and uncorr_brng <= brng_h:
    
        # get the lower and upper limits to lineairize
        for indx in range(0,no_elements - 1):
            (brng_l, corr_brng_l) = brng_corr_table[indx]
            (brng_h, corr_brng_h) = brng_corr_table[indx+1]
            if uncorr_brng >= brng_l and uncorr_brng <= brng_h:
                break

        # get the data for the linear equation for this segment
        #print (f'brng_l = {brng_l} corr_brng_l = {corr_brng_l}')
        #print (f'brng_h = {brng_h} corr_brng_h = {corr_brng_h}')
         
        # corr_brng_l should always be less than corr_brng_h, for base_indx 28 this is not the case
        #if corr_brng_l > corr_brng_h:
        #    corr_brng_l = corr_brng_l % 360.0
               
        # approximate the bearing by using a linear equation
        corr_brng = ((corr_brng_h - corr_brng_l)/(brng_h - brng_l) * (uncorr_brng - brng_l) + corr_brng_l) % 360.0
        #print (f'corr_brng = {corr_brng}')
        
    else:
        # uncorr_brng is outside allowable range
        corr_brng = -1
        print (f'Error - uncorr_brng = {uncorr_brng} outside expected range')
        
    return(corr_brng)
    






# get compass bearings
def get_bearings(bus:int, mux_buses: list, magsensors:list) -> list:
    
    my_name = 'get_bearings()'
    
    bearings = []
    
    #dbg.write(f'{my_name} MUX_BUSES = {my_gpio.MUX_BUSES}\n')
    #dbg.write(f'{my_name) magsensors = {magsensors}\n')

    for mux_bus, magsensor in zip(mux_buses, magsensors):
        
        # select the mux bus of this compass
        select_mux_bus(bus, my_gpio.MUX_ADR, mux_bus)
        
        # get the bearing of this compass
        bearings.append(magsensor.get_bearing())  
        
    return(bearings)


# select through the I2C multiplexer the requested multiplexed bus
def select_mux_bus(bus, mux_addr, mux_bus):

    my_name = 'select_mux_bus()'
    
    
    reg_cmd = 1         # command register of the TCA9548 
    
    # the mux buses 1-8 are selected by selecting bit respective 00000001 through 10000000
    value = 2 ** mux_bus
        
    # need to create a bytearray with 1 element from variable 'value'
    b_value = bytearray()
    b_value.append(value)

    # must be a list of bytes
    l_value = list(b_value)

    # write the register
    bus.write_i2c_block_data(mux_addr, reg_cmd, l_value)
    
    return ()



# convert a 2D vector to euler angle (bearing)
def vector2d_bearing(x:float,y:float)->float:
     
    eul: float = 0.0
    deg: float = 0.0
    
    # prevent dividing by 0.0
    if in_range(x, -0.001, 0.001):
        if y >= 0.0:
            eul = 0.0                       # vector (0,0) is invalid but will return 0
        else:
            eul = 180.0
    else:        
        deg = math.degrees(math.atan(y/x))        
        if x<0:
            deg += 180.0                    # fixed mirrored angle of arctan
        eul = 360.0 - (270.0+deg)%360.0     # folded to [0,360) domain
    
    return (eul)



# convert a euler angle (bearing) to a 2D vector
def bearing_vector2d(b:float):
    
    my_name = 'bearing_vector2d()'
     
    eul: float = 0.0
    deg: float = 0.0
    
    if b<=90:
        eul = 90 - b
    else:
        eul = 450.0 - b
            
    print (f'{my_name} b = {b} eul = {eul}')
    
    # return the vector
    x = math.sin(math.radians(b))       # this is ugly but works
    y = math.cos(math.radians(b))
    
    return (x,y)
    

def calc_avg_bearing(bearings:list) -> float:
  
    my_name = 'calc_avg_bearing'
    
    x_total: float = 0.0
    y_total: float = 0.0
    avg_bearing: float = 0.0
    
    # consider the bearings as vectors (x,y) and add the vectors
    # the result vector will be the average (mean)
    for bearing in bearings:
        
        # convert from bearing to vector
        (x,y) = bearing_vector2d(bearing)
        
        # add up the vectors
        x_total += x
        y_total += y
        
    #print (f'{my_name}: x_total = {x_total} y_total = {y_total}') 
   
    avg_bearing = vector2d_bearing(x_total,y_total)
    print (f'{my_name}: avg_bearing = {avg_bearing}')
    
    return(avg_bearing)
    
    

        

# add two bearings
def brng_add(bearing_1: float, bearing_2: float) -> float:
    bearing_sum = (bearing_1 + bearing_2) % 360.0
    return (bearing_sum)
    

# subtract two bearings
def brng_diff(bearing_1: float, bearing_2: float) -> float:

    r = (bearing_2 - bearing_1) % 360.0
    # Python modulus has same sign as divisor, which is positive here,
    # so no need to consider negative case
    if r >= 180.0:
        r -= 360.0
    return (r)

 
 
    
# wait until the drive or zero turn command is completed		
def wait_cmd_done():

    my_name = 'wait_cmd_done()'
    
    # need to sleep for a brief moment to allow the arduino driver to process the cmd
    time.sleep(0.5)
    
    # wait until status indicates completed
    cmd_finished = GPIO.input(my_gpio.WHL_DRV_PIN)
    while not cmd_finished:
        dbg.write(f'{my_name} loop cmd_finished = {cmd_finished}\n')
        time.sleep(0.5)
        cmd_finished = GPIO.input(my_gpio.WHL_DRV_PIN)

    return



# turn to the requested bearing and drive
def turn_drive(req_bearing: float, dist_cm: float, mode: int) -> float:

    # mode - manual_mode 
    #      - auto_mode

    my_name = 'turn_drive()'
    
    # motor reduction = 60:1, reduction 102:18, sprockets 12/15, wheel diameter 29.5 cm
    # 1 step (revolution) = 29.5 * PI / 60 * 12/15 * 18/102 = 0.218063 cm
    steps_cm   = 4.58582       # wheel: steps per cm
    
    # get the absolute bearing the mower is pointing to
    current_bearing = get_compass_bearing(compasses)
    
    # in manual mode, the mower is operated from the hmi directly and req_bearing is the requested turn angle
    # either positive or negative (counter clockwise)
    if mode == manual_mode:
        # calculate the requested bearing
        req_bearing = brng_add(current_bearing,req_bearing)
    
    dbg.write(f'{my_name}: mode = {mode} current_bearing = {current_bearing:.2f} req_bearing {req_bearing:.2f}\n')
       
    # correct if necessary to make sure we are going in the right direction; with
    # a turning circumferance of 153.9cm, 1 degree will be 0.427 cm.  1 step is 0.218 cm  
    # do not correct any less than 2 degrees (to avoid over correcting). 
    cnt = 0
    while abs(brng_diff(current_bearing, req_bearing)) >= 2.0:
    
        # only try a few times
        cnt += 1
        
        # calculate the angle to turn
        turn_angle = brng_diff(current_bearing, req_bearing)
        dbg.write(f'{my_name}: cnt = {cnt} current_bearing {current_bearing:.2f} req_bearing {req_bearing:.2f} turn_angle = {turn_angle:.2f}\n')
        
        # calculate the distance the wheels need to travel
        turn_cm = turn_angle  / 360.0 * math.pi * mower_width
        turn_steps = int(turn_cm * steps_cm)
        
        # zero turn at 100% duty
        mower_cmd('z,' + str(turn_steps) + ',0,100')
        wait_cmd_done()
    
        # check if we are close
        current_bearing = get_compass_bearing(compasses)
    
        if cnt > 10:
            dbg.write(f'{my_name}: current_bearing = {current_bearing:.2f} unable to reach req_bearing = {req_bearing:.2f}\n')
            blade_motor_control(blade_enabled, my_gpio.OFF)
            exit()
            break
    
    # calculate the steps to drive
    drive_steps = int(dist_cm * steps_cm)

    # drive at 100% duty    
    mower_cmd('d,' + str(drive_steps) + ',0,100')
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
    
    dbg.write(f'{my_name} command = {command}\n')


    # drive command: d,<steps>,<steer_delta>,<duty>
    if command[:1] == 'd':          # slice first character of command
	
        cmd_ln = command.split(',')
        if len(cmd_ln) < 4:
                dbg.write("Enter d,<steps>,<steer_delta>,<duty>\n")
        else:
                # setup the drive registers (6 bytes)
                b_reg_drive = bytearray(6)
                
                steps = int(cmd_ln[1])
                struct.pack_into('<h', b_reg_drive, 0, steps)              # short integer 2 bytes

                steer_steps = int(cmd_ln[2])
                struct.pack_into('<h', b_reg_drive, 2, steer_steps)        # short integer 2 bytes

                duty = int(cmd_ln[3])
                struct.pack_into('<h', b_reg_drive, 4, duty)               # short integer 2 bytes
                dbg.write(f'{my_name} b_reg_drive\n')

                # convert to list
                l_reg_drive = list(b_reg_drive)

                # Writes 1+6 bytes, first byte is the register address to write into
                bus.write_i2c_block_data(my_gpio.ARD_ADDR, reg_drive, l_reg_drive)

                # Write the drive command; this will be reset by the slave
                cmd_char = 0x64
                list1 = [cmd_char]
                bus.write_i2c_block_data(my_gpio.ARD_ADDR, reg_cmd, list1)
                
    # zero turn command: z,<steps>,0,<duty>
    elif command[:1] == 'z':          # slice first character of command

        cmd_ln = command.split(',')
        if len(cmd_ln) < 4:
                dbg.write(" Enter z,<steps>,0,<duty>\n")
        else:
                # setup the drive registers (6 bytes)
                b_reg_drive = bytearray(6)
                
                steps = int(cmd_ln[1])
                struct.pack_into('<h', b_reg_drive, 0, steps)              # short integer 2 bytes

                steer_steps = 0
                struct.pack_into('<h', b_reg_drive, 2, steer_steps)        # short integer 2 bytes

                duty = int(cmd_ln[3])
                struct.pack_into('<h', b_reg_drive, 4, duty)               # short integer 2 bytes
                dbg.write(f'{my_name} b_reg_drive\n')
                
                dbg.write(f'{my_name} z command steps = {steps} steer_steps = {steer_steps}\n')
                # convert to list
                l_reg_drive = list(b_reg_drive)

                # Writes 1+6 bytes, first byte is the register address to write into
                bus.write_i2c_block_data(my_gpio.ARD_ADDR, reg_drive, l_reg_drive)

                # Write the zero turn command; this will be reset by the slave
                cmd_char = 0x7A
                list1 = [cmd_char]
                bus.write_i2c_block_data(my_gpio.ARD_ADDR, reg_cmd, list1)
    
    # Read command back: c
    elif command[:1] == 'c':                    # slice first character of command
    
        # read 1 byte containing the command
        l_reg_cmd = bus.read_i2c_block_data(my_gpio.ARD_ADDR, reg_cmd, 1)
        
        # convert from list to bytearray
        b_reg_cmd = bytearray(l_reg_cmd)
        data = struct.unpack('<B', b_reg_cmd)    # unsigned int 1 byte
        dbg.write(f'{my_name} cmd data = {data}\n')
        

    # tune command: t,<Kp>,<Ki>,<Kd>
    elif command[:1] == 't':                    # slice first character of command
	
        cmd_ln = command.split(',')
        if len(cmd_ln) < 4:
                dbg.write("Enter t,<Kp>,<Ki>,<Kd>\n")
        else:
                # setup the drive registers (12 bytes)
                b_reg_tune = bytearray(12)
                
                kp = float(cmd_ln[1])
                struct.pack_into('<f', b_reg_tune, 0, kp)            # float, 4 bytes

                ki = float(cmd_ln[2])
                struct.pack_into('<f', b_reg_tune, 4, ki)            # float, 4 bytes

                kd = float(cmd_ln[3])
                struct.pack_into('<f', b_reg_tune, 8, kd)            # float, 4 bytes
                dbg.write(f'{my_name} b_reg_tune = {b_reg_tune}\n')

                # convert to list
                l_reg_tune = list(b_reg_tune)

                # Writes 1+12 bytes, first byte is the register address to write into
                bus.write_i2c_block_data(my_gpio.ARD_ADDR, reg_tune, l_reg_tune)

                # Signal the tune command is ready; this will be reset by the slave
                cmd_char = 0x74
                list1 = [cmd_char]
                bus.write_i2c_block_data(my_gpio.ARD_ADDR, reg_cmd, list1)

        
    # show command: s
    elif command[:1] == 's':                    # slice first character of command
            
        # read 3 x 4 bytes containing the tuning parameters
        l_reg_tune = bus.read_i2c_block_data(my_gpio.ARD_ADDR, reg_tune, 12)
        
        # convert from list to bytearray
        b_reg_tune = bytearray(l_reg_tune)
        data = struct.unpack('<fff', b_reg_tune)    # float, 3 x 4 bytes
        dbg.write(f'{my_name} tuning data =  {data}\n')
        
        # let i2c bus recover
        time.sleep(0.1)
        
        # read 2 x 2 bytes containing the motor counters
        l_reg_motor = bus.read_i2c_block_data(my_gpio.ARD_ADDR, reg_motor, 4)
        
        # convert from list to bytearray
        b_reg_motor = bytearray(l_reg_motor)
        data = struct.unpack('<hh', b_reg_motor)    # short integer
        dbg.write(f'{my_name} motor_data =  {data}\n')
                
                
    # real all registers command: r
    elif command[:1] == 'r':                    # slice first character of command
    
        # read the entire register back from the Arduino
        data = bus.read_i2c_block_data(my_gpio.ARD_ADDR, reg_cmd, reg_next)
        dbg.write(f'{my_name} all registers =  {data}\n')
         
    # quit
    elif command == 'q':
        exit()
	
    return(data)

# controls the blade motor
def blade_motor_control(blade_enabled:bool, req_state: bool):

    my_name = 'blade_motor_control():'
    
    if blade_enabled:
        # switch the blade motor on or off
        GPIO.output(my_gpio.BLD_MOT_PIN, req_state)
    else:
        dbg.write(f'{my_name} Blade motor not enabled\n')
    return()
    
        


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
     
    #dbg.write(f'{my_name} segments_0 = {segments_0}\n')
    
    
    
    
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
    
    #dbg.write(f'{my_name} segments_1 = {segments_1}\n')
   
   
   
   
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
    
    #dbg.write(f'{my_name}: work_path = {work_path}\n')
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
    #dbg.write(f'{my_name} dx = {dx} dy = {dy}\n')
    
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




def traditional_pattern(camera, work_area_limits, res_q):

    my_name = 'traditional_pattern()'

    dbg.write(f'{my_name}: not implemented yet\n')
    
    return()
    


def spiral_pattern(camera, work_area_limits, res_q):
    
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
    for bp in beacon_pos_cm:
        #unpack
        b_no, b_x, b_y = bp

        # build the dictionary
        bcn_data[b_no] = Beacon (bcn_no = b_no,  a = b_x,   b = b_y, r = 0.0)
    dbg.write(f'{my_name}: bcn_data = {bcn_data}\n')


    # get the vehicle position, note that the vehicle bearing may not be known yet
    (xs,ys,veh_brng) = veh_pos_cm
    
    # get the current vehicle bearing
    veh_brng = get_compass_bearing(compasses)
    dbg.write(f'{my_name}: veh_brng {veh_brng:.2f}\n')
    
    # send the updated vehicle position to the hmi
    veh_pos_cm = (xs,ys,veh_brng)
    que.push_queue(res_q, que.t_vehicle_position, veh_pos_cm)
    
    dbg.write(f'{my_name}: work_area_limits = {work_area_limits}\n')
    dbg.write(f'{my_name}: veh_pos_cm = {veh_pos_cm}\n')
    
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
            dbg.write(f'\n\n{my_name}: seg {seg}\n')           
                
            # prepare the path for the next interation
            new_path.append((xs,ys))

            # get the current vehicle position
            (x_veh, y_veh, veh_brng) = veh_pos_cm
            dbg.write(f'{my_name}: This should be my position - veh_pos_cm = {veh_pos_cm}\n')
            
            # check the current position by triangulation
            position_found, tri_lat = get_veh_position(camera, bcn_data)
            if position_found: 
                (x_veh, y_veh) = tri_lat.centroid.x, tri_lat.centroid.y 
                dbg.write(f'{my_name}: 1. Position calculated by triangulation: ({x_veh:.2f},{y_veh:.2f}) - {veh_brng:.2f}\n')
                msg = "1. Position calculated by triangulation: " + str(f'{x_veh:.2f}') + ' ' + str(f'{y_veh:.2f}') + ' bearing ' + str(f'{veh_brng:.2f}') + ' ' + str(datetime.datetime.now())
                que.push_queue(res_q, que.t_message, msg)
            else:
                dbg.write(f'{my_name}: No beacons detected\n')
                msg = "1. No beacons detected " + str(datetime.datetime.now())
                que.push_queue(res_q, que.t_message, msg)
            
            # check if we need to drive to the start position
            # generally this should be a small correction, but when starting a new pattern it could be a long distance
            #if (x_veh != xs) or (y_veh != ys):
            if not(in_range((x_veh - xs), -accuracy_cm, accuracy_cm) and in_range((y_veh - ys), -accuracy_cm, accuracy_cm)):

                # get bearing and distance to (xs, ys) from where we are
                (start_brng, start_dist_cm) = calc_course(x_veh, y_veh, xs, ys)
                
                # zero turn and drive to the start position
                dbg.write(f'{my_name}: Moving to start position - bearing {start_brng:.2f} distance {start_dist_cm:.2f} --- ({x_veh:.2f},{y_veh:.2f}) to ({xs:.2f},{ys:.2f})\n')
                veh_brng = turn_drive(start_brng, start_dist_cm, auto_mode)
                total_distance += start_dist_cm
                
                # check if arrived at the start position; check by triangulation
                position_found, tri_lat = get_veh_position(camera, bcn_data)
                if position_found: 
                    (x_veh, y_veh) = tri_lat.centroid.x, tri_lat.centroid.y 
                    dbg.write(f'{my_name}: 2. Position calculated by triangulation: ({x_veh:.2f},{y_veh:.2f}) - {veh_brng:.2f}\n')
                    msg = "2. Corrected xs,ys position check by triangulation: " + str(f'{x_veh:.2f}') + ' ' + str(f'{y_veh:.2f}') + ' bearing ' + str(f'{veh_brng:.2f}') + ' ' + str(datetime.datetime.now())
                    que.push_queue(res_q, que.t_message, msg)
                    
                    # in case we are still not exactly at xs,ys, calculate a new heading and travel distance to end up in xe,ye
                    (abs_brng, trvl_dist_cm) = calc_course(x_veh, y_veh, xe, ye)
                    xs = x_veh
                    ys = y_veh
                    veh_brng = abs_brng
                    dbg.write(f'{my_name}: 2. Updated course from xs,ys ({xs},{ys}) to xe,ye ({xe},{ye}): abs_brng = {abs_brng} trvl_dist_cm = {trvl_dist_cm}\n') 
                    
                else:
                    dbg.write(f'{my_name}: 2. No beacons detected\n')
                    msg = "2. xs,ys position check - No beacons detected " + str(datetime.datetime.now())
                    que.push_queue(res_q, que.t_message, msg)
                
            # send the vehicle position to the hmi 
            veh_pos_cm = (xs,ys,veh_brng)
            que.push_queue(res_q, que.t_vehicle_position, veh_pos_cm)
            dbg.write(f'{my_name}: Now at start position {veh_pos_cm}\n')
                
            # zero turn to the required bearing to xe, ye
            dbg.write(f'{my_name}: Zero turning to {xe, ye} - abs bearing {abs_brng:.2f}\n')

            # mow the segment (xs,ys) to (xe,ye) 
            blade_motor_control(blade_enabled, my_gpio.ON)
            dbg.write(f'{my_name}: Now mowing to {xe:.2f}, {ye:.2f}\n')
            veh_brng = turn_drive(abs_brng, trvl_dist_cm, auto_mode)
            total_distance += trvl_dist_cm
            blade_motor_control(blade_enabled, my_gpio.OFF)
            
            # send the vehicle position to the hmi 
            veh_pos_cm = (xe,ye, veh_brng)
            que.push_queue(res_q, que.t_vehicle_position, veh_pos_cm)
            
            dbg.write(f'{my_name}: 3. now at position {veh_pos_cm}\n')
            
        # save the circumference
        last_circumf = circumf
        
        # calculate the new work area limits
        #dbg.write(f'{my_name} new_path = {new_path}\n')
        work_area_limits = []
        work_area_limits = calc_work_path(new_path)   
        
        # calculate the circumference of the new work area
        circumf = 0
        for seg in work_area_limits:
            (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm) = seg
            circumf = circumf + trvl_dist_cm 
        
        # calculate the change in circumference
        delta_circumf = last_circumf - circumf
        dbg.write(f'{my_name}: circumf:  {circumf:.2f}  {delta_circumf:.2f}\n')
 
    # announce spiral pattern has completed
    msg = "Spiral pattern completed at " + str(datetime.datetime.now())
    que.push_queue(res_q, que.t_message, msg)
    msg = "Total distance: " + str(f'{total_distance:.2f}')
    que.push_queue(res_q, que.t_message, msg)
                   
    return()
    
     
    
def random_pattern(camera, work_area_limits, res_q):

    my_name = 'random_pattern()'

    dbg.write(f'{my_name}: not implemented yet\n')
    
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
    global compasses                    # instances containing compass data
    global dbg                          # debug log file handle
    
    my_name = 'mowing()'
    
 
    
    # compass calibration data 13/5/2023 - with Mirror assemble moved to wheel axle
    calibration_data =  {   5: [[1.040015182688284, -0.023941176664552588, -479.12673626694095], [-0.023941176664552588, 1.014324061558043, -402.73170503879896], [0.0, 0.0, 1.0]],
                            6: [[1.0516924554923728, -0.03363453657900073, 746.6311816526782], [-0.03363453657900073, 1.0218848580534359, 275.9040953542344], [0.0, 0.0, 1.0]],
                            7: [[1.0520519392439893, -0.031777234227061135, -585.5233475597431], [-0.03177723422706108, 1.0193997117069584, -173.53954881336387], [0.0, 0.0, 1.0]] }
    
    work_area_limits = []       # list of work area limits per segment;
                                # (xs, ys, m, b, xe, ye, abs_brng, trvl_dist_cm)
    
    # start the debug log
    try:
        dbg = open(mowing_debug_log,'w')
    except:
        dbg.write(f'{my_name} cannot open {mowing_debug_log}, exiting\n')
        exit
    
    
    # setup the I2C bus
    bus = smbus.SMBus(my_gpio.I2C_CH)

    # give the I2C device time to settle
    time.sleep(2)
    
    # set up the globals for the compasses on I2C mux_buses 5, 6, 7
    for mux_bus in my_gpio.MUX_BUSES:
        
        # create an instance of each compass and store the instances in a global
        select_mux_bus(bus, my_gpio.MUX_ADR, mux_bus)  
        compass = py_qmc5883l.QMC5883L()
        compasses.append(compass)
        
        # and enter the calibration
        compass.calibration = calibration_data[mux_bus]

    dbg.write(f'{my_name} compasses = {compasses}\n')
    dbg.write(f'{my_name} QMC8553L library passed\n')

    # initialize the camera; it needs 2 seconds to start-up; calibrating the mirror will do that
    # the PiCamera library allows setting the camera up in advance resulting in faster images
    camera = Picamera2()
    
    # cannot use custom square resolutions anymore
    config = camera.create_still_configuration(main={"format": "XRGB8888", "size": (frm_dim['x'], frm_dim['y'])}, buffer_count=1)
    camera.configure(config)
    camera.start()
    time.sleep(2)
    
    # rotate the mirror clock-wise until the calibration point is found
    calibrated = calibrate_mirror()
    if not calibrated:
        dbg.write(f'{my_name}: mirror calibration point not found after 1 full rotation\n')
        GPIO.cleanup()
        exit(0)
    else:
        
        # calibration point was found, now rotate the mirror to face front
        # the number of steps to rotate is determined by the physical contruction and is adjusted to face the front.
        rotate_cam(611, my_gpio.CCW)                
        dbg.write(f'{my_name}: zero point reached\n')
        
            
    while True:
                               
        # get any commands from the hmi
        pkgs = que.get_queue(cmd_q)
        dbg.write(f'{my_name} mower thread pkgs received= {pkgs} gd.kill_thread_1 = {gd.kill_thread_1}\n')

        for pkg in pkgs:
        
            dbg.write(f'{my_name}: pkg received {pkg}\n')
            
            (pkg_time, ptype, data) = pkg
            now = time.time()
            if now - pkg_time < que.expire_threshold:


                # 'get position' command from hmi
                if ptype == que.t_get_position:
                    # 1) this is not ideal, bcn_data is setup in spiral pattern and is currently duplicated for getting the position
                    # unpack the data
                    (beacon_pos_cm) = data
                    
                    # setup the beacon data dictionary from global variable beacon positions collected in the main
                    bcn : Beacon
                    bcn_data = {}
                    for bp in beacon_pos_cm:
                        #unpack
                        b_no, b_x, b_y = bp

                        # build the dictionary
                        bcn_data[b_no] = Beacon (bcn_no = b_no,  a = b_x,   b = b_y, r = 0.0)
                    dbg.write(f'{my_name}: bcn_data = {bcn_data}\n')
    
                    # get the current vehicle bearing
                    veh_brng = get_compass_bearing(compasses)
    
                    # check the current position by triangulation
                    position_found, tri_lat = get_veh_position(camera, bcn_data)
                    if position_found:
             
                        (x_veh, y_veh) = tri_lat.centroid.x, tri_lat.centroid.y 
                        veh_pos_cm = (x_veh, y_veh, veh_brng)
                        dbg.write(f'{my_name}: 0: Position calculated by triangulation: veh_pos_cm {veh_pos_cm}\n')
                        
                        # update the log in the hmi
                        msg = "0: Position calculated by triangulation: " + str(f'{x_veh:.2f}') + ' ' + str(f'{y_veh:.2f}') + ' bearing ' + str(f'{veh_brng:.2f}') + ' ' + str(datetime.datetime.now())
                        que.push_queue(res_q, que.t_message, msg)

                        # update the map in the hmi with the updated vehicle position
                        que.push_queue(res_q, que.t_vehicle_position, veh_pos_cm)
    
                    else:
                        dbg.write(f'{my_name}: 0: No beacons detected\n')
                        msg = "0: No beacons detected " + str(datetime.datetime.now())
                        que.push_queue(res_q, que.t_message, msg)
            
        
                # 'start mowing pattern' command from hmi
                if ptype == que.t_start_mowing:

                    # unpack the start vehicle position, mow pattern, the work area and beacon positions
                    (veh_pos_cm, mowing_pattern, work_area_coord_cm, beacon_pos_cm) = data
                    dbg.write(f'{my_name} veh_pos_cm = {veh_pos_cm} mowing_pattern = {mowing_pattern} beacon_pos_cm = {beacon_pos_cm}\n')
                 
                    
                    # from the given work area, calculate the coordinates and equations that the mower should adhere to
                    work_area_limits = calc_work_path(work_area_coord_cm)

                    # signal to allow the mowing pattern, this global variable is checked by the relevant routines 
                    stop_mowing = False
                     
                    # and start the pattern, pass the response queue to allow dat ato be posted back to the hmi
                    if mowing_pattern == gd.mow_traditional:             
                        traditional_pattern(camera, work_area_limits, res_q)
                    else:
                        if mowing_pattern == gd.mow_spiral:
                            spiral_pattern(camera, work_area_limits, res_q)
                        else:
                            if mowing_pattern == gd.mow_random:
                                random_pattern(camera, work_area_limits, res_q)
                     

                                
                
                # stop mowing pattern command from hmi, this package type has no data
                if ptype == que.t_stop_mowing:

                    # signal to stop the mowing pattern immediately, this global variable is monitored by the relevant functions
                    stop_mowing = True
                    
                        
                # manual command
                if ptype == que.t_man_mv_forwrd or ptype == que.t_man_mv_cntclk or \
                   ptype == que.t_man_mv_clk    or ptype == que.t_man_mv_bckwrd:    
                       
                    (turn_angle,dist_cm) = data
                    dbg.write(f'{my_name} manual cmd - req turn_angle = {turn_angle:.2f} req distance = {dist_cm:.2f}\n')
                    turn_drive(turn_angle, dist_cm, manual_mode)                              
                
                                        
                # manual Stop command
                if ptype == que.t_man_mv_stop:
                    (turn_angle,dist_cm) = data
                    dbg.write(f'{my_name} Stop cmd\n')
            
            
        # check if this thread needs to exit
        if gd.kill_thread_1: 
            
            #  release I2C bus
            bus.close()
            bus = None  
            
            # close the debug log file
            dbg.close()    
            return()
        
        # sleep for 1 second
        time.sleep(1)
            
                    
    return()
