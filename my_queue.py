# !/usr/bin/python3

import queue
import time

# package types for all threads
t_message = 1
t_vehicle_start_position = 2
t_vehicle_position_tri = 3
t_beacon_position = 4
t_start_mowing = 5
t_stop_mowing = 6
t_beacon_status = 7
t_video_frame = 8
t_vehicle_position = 9
t_man_mv_forwrd = 10
t_man_mv_cntclk = 11
t_man_mv_clk = 12
t_man_mv_bckwrd = 13
t_man_mv_stop = 14

expire_threshold = 60        # threshold in seconds when queue pkg expire

    
# push a package on the specified queue
def push_queue(q, type, data):

        # get the current time in order to time stamp the package
        now = time.time()
        
        # create the item to add to the queue as a immutable tuple
        pkg = (now, type, data)
        q.put(pkg)


        
# get all data from the specified queue      
def get_queue(q):
    
    pkgs = []       # the list of packages in the queue
    
    # get all packages of the queue
    while True:  
    
        try:
            pkg = q.get(False)  # don't wait for the queue
    
            # append it to the list of packages received
            pkgs.append(pkg)
    
        # in case the queue is empty
        except queue.Empty:
            break
            
    # return the list of packages
    return(pkgs)
