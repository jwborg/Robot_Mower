# !/home/pi/.virtualenvs/cv/bin/python

import numpy as np
import math
import time


# application imports
import my_queue as que
    
# global variables - shared outside this thread

# threads
kill_thread_1 = False       # set by toggle_mower_thread()

# global variables - not shared outside this thread
mower_width = 25            # half mower width in cm
work_area = []              # list of work area data per segment  (x,y, m, b)





# the work area is given in coordinates in cm. Now calculate the coordinates that
# the mower should adhere to due to it's width and the linear equations of each of the segments 
# of the work area
def calc_work_area(work_area_coord_cm):

    global work_area
    
    segments_0 = []         # linear equation data of each of the segments
    segments_1 = []         # start coord and linear equation data of each of the segments
    first_coord = None
    first_segment = None
    
    
    
    
    for coord in work_area_coord_cm:
            
        # first time, no segments defined yet
        if not first_coord == None:
        
            # determine the linear equation of this segment
            (m, b, b_in) = get_lin_eq_segment(last_coord, coord, mower_width)
            
            # store only the inner boundary
            segments_0.append((m, b_in))
        else:
            # first coordinate, store it
            first_coord = coord
            
        # save coord for the next segment
        last_coord = coord
        
    # now create the last segment by connecting to the first coordinates
    (m, b, b_in) = get_lin_eq_segment(last_coord, first_coord, mower_width)
    
    # store only the inner boundary
    segments_0.append((m, b_in))        
     
    #print ('calc_work_area: segments_0 = ', segments_0)
    
    
    
    
    # calculate the crossing coordinates of each of the segments
    for segment in segments_0:
    
        # need to unpack to store the segment in work data
        (m, b_in) = segment
    
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
    (m, b_in) = first_segment
    (x,y) = calc_segment_cross(last_segment, first_segment)
    segments_1.append((x, y, m, b_in))
    
    #print ('calc_work_area: segments_1 = ', segments_1)
   
   
   
   
    # add the end coordinate of each segment, by copying it from the start coordinate of the next segment
    no_segments = len(segments_1)
    for index, segment in enumerate(segments_1):
 
        # unpack
        (xs, ys, m, b) = segment
        
        # the start point of the next segment is the end point of this segment
        if index == no_segments - 1:
            (xe, ye, _, _) = segments_1[0]               # the last segment uses the start coord of the first segment
        else:
            (xe, ye, _, _) = segments_1[index + 1]       # all other segments use the start coord of the next segment

        # now build the final work area data list
        work_area.append((xs, ys, m, b, xe, ye))
    
    print ('calc_work_area: work_area = ', work_area)
    

    
    

# determine linear equation y = mx + b of line through 2 coordinates (x1, y1) and (x2, y2). Then return 
# the linear equation of the innner boundary. This basically the same line, but now with a vertical offset, 
# therefore a different b.
def get_lin_eq_segment (x1y1, x2y2, mower_width):

    # unpack
    (x1, y1) = x1y1
    (x2, y2) = x2y2
    
    # check for vertical segments, which would give a divide by 0
    if math.fabs(math.isclose(x1, x2, rel_tol=1e-2)):

        # vertical segment
        if y2 > y1:
            m = float('inf')
        else:
            m = float('-inf')
        
        # need to be initialized, but not used
        b = 0
        b_inner = 0
    else:
    
        # all lines other than vertical
        m = (y2 - y1) / (x2 - x1)
        b = y1 - m * x1
        
        # the inner boundery line is y = mx + b plus or minus yw,  where yw is the y-component of the mower width 
        yw = mower_width / math.cos( math.atan((y2- y1) / (x2 - x1)))
        
        # depending on the line segment direction, add or subtract yw
        if (x2 - x1) > 0:
            b_inner = b - yw
        else:
            b_inner = b + yw
           
    return (m, b, b_inner)
 

 
# calculate the coordinate where 2 segments (y = mx + b) cross 
def calc_segment_cross(le1, le2):

    # unpack
    (m1, b1) = le1
    (m2, b2) = le2
 
    # m1 * x1 + b1 = m2 * x2 + b2
    x = (b2 - b1) / (m1 - m2)
    y = m1 * x + b1
        
    return (x,y)


    
# the main for thread mowing
def mowing(cmd_q, res_q):

    global start_veh_pos_cm             # start position of vehicle in cm
    global veh_pos_cm                   # the vehicle position in cm
    global veh_pos_tri_cm               # the vehicle position as determined by triangulation
    global mowing_pattern               # the mowing pattern requested
    global work_area_coord_cm           # the work area coordinates in cm    

    work_area_cm = []
        
    while True:
        
        print ('\n\nkill_thread_1 = ', kill_thread_1)
        
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
                    (start_veh_pos_cm, mowing_pattern, work_area_coord_cm) = data
                    print ('mowing: start_veh_pos_cm = ', start_veh_pos_cm, 'mowing_pattern = ', mowing_pattern)
                    print ('mowing: work_area_cm = ', work_area_coord_cm)
                    
                    # from the given work area, calculate the coordinates and equations that the mower 
                    # should adhere to
                    calc_work_area(work_area_coord_cm)
        
                # update of the vehicle position as detected by trangulation
                if type == que.t_vehicle_position_tri:

                    # unpack the vehicle position as detected by triangulation
                    (veh_pos_tri_cm) = data
                    print ('mowing: veh_pos_tri_cm = ', veh_pos_tri_cm)
        
        
                  
        # check if this thread needs to exit
        if kill_thread_1:        
            return()
        
        # sleep for 1 second
        time.sleep(1)
            
                    
    return()
