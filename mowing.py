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
