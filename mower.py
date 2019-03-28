#!/usr/bin/python

from tkinter import *
from tkinter import ttk
from tkinter import filedialog


import cv2
import numpy as np

import math
import time
import threading
import queue
import tkinter.font # bold

# thread imports
import position as pos
import my_queue as que

# global variable shared outside main
pos.kill_thread0 = False                        # flag to kill thread 0 


# global variables within main - not shared with threads

debug_level = 0
version = 1.0
my_name = 'mower.py'

# canvas size in pixels
p_x_sz = 500
p_y_sz = 500
cm_pix = 3                                      # scale_factor (cm per pixel)

    
# beacons
sv_beacons = []                                 # screen variables for beacons
b_set_veh_start_pos = False                     # state of the set vehicle button
beacon_index = 0                                # the beacon to be changed [0-2]
beacon_coord = np.array([(0,0),(0,0),(0,0)])    # (x,y) coordinates of beacons 1, 2 and 3 in pixels
beacon_pos = np.array([(0,0),(0,0),(0,0)])      # (x,y) coordinates of beacons 1, 2 and 3 in cm
    
# work area
b_set_work_area = False
work_area_coord = []                            # (x,y) coordinates of work area in pixels
last_work_area_coord = []                       # (x,y) coordinates of last work arae in pixels

# starting and stopping of threads
b_start_pos = False                             # state of stop/start button     

# vehicle
veh_pos = (0,0)                                 # vehicle position (x,y) in pixels; (0,0) lower left cormer
veh_color = 'red'

# create the canvas area
def create_canvas_area():

    # create the main frame
    frm_cnvs = ttk.Frame(content, padding=(20, 20, 20, 20 ))
    frm_cnvs.grid(column=0, row=0, sticky=(N,W,S,E))
    frm_cnvs.grid_columnconfigure(1, weight=1)
    frm_cnvs.grid_rowconfigure(1, weight=1)

    # create the canvas to draw
    cnvs = Canvas(frm_cnvs, bg="red", height=p_x_sz, width=p_y_sz)
    cnvs.grid()

    # capture the left mouse button when clicked
    cnvs.bind("<Button-1>", cnvs_left_click)   

    return (frm_cnvs, cnvs)

        
        
def create_status_area():

    global sv_beacons
    global sv_veh_pos
    global b01_entry
    global sv_b01_dist
    
    # create the main frame
    frm_sts = ttk.Frame(content, padding=(20, 20, 20, 20 ))
    frm_sts.grid(column=1, row=0, sticky=(N,W,S,E))
    frm_sts.grid_columnconfigure(3, weight=1)
    frm_sts.grid_rowconfigure(20, weight=1)
    
    # create the beacon label
    Label( frm_sts, text='Beacons').grid(column=0, row=0, sticky=(N,W))
    
    # create the beacon coordinates string vars
    for var in range(3):
        sv_beacon = StringVar(value = '--- , ---')
        lbl_beacon1 = Label(frm_sts, textvariable=sv_beacon).grid(column=1, row=var)
        sv_beacons.append(sv_beacon)

    # create the beacon label
    Label( frm_sts, text='Veh Pos').grid(column=0, row=5, sticky=(N,W))

    # create the vehicel position string variable
    sv_veh_pos = StringVar(value = '--- , ---')
    lbl_veh_pos = Label(frm_sts, textvariable=sv_veh_pos).grid(column=1, row=5)
    
    # create the button to position the vehicle
    btn_set_veh_start_pos = ttk.Button(frm_sts, text ="Set Mower Start Position", command = set_veh_start_pos)
    btn_set_veh_start_pos.grid(column=2, row=5, sticky=(N,W))
    
    # create the work area label
    Label( frm_sts, text='Work Area').grid(column=0, row=6, sticky=(N,W))
    
    # create the button to set the work area
    btn_set_work_area = ttk.Button(frm_sts, text ="Set Work Area", command = set_work_area)
    btn_set_work_area.grid(column=2, row=6, sticky=(N,W))
        
    # create the Positioning label
    Label( frm_sts, text='Positioning').grid(column=0, row=7, sticky=(N,W))
    
    # create the button to start/stop positioning
    btn_toggle_pos = ttk.Button(frm_sts, text ="Start", command = toggle_position_thread)
    btn_toggle_pos.grid(column=2, row=7, sticky=(N,W))
    
    return (frm_sts, btn_set_veh_start_pos, btn_set_work_area, btn_toggle_pos)

    
def create_logging_area():

    # create the main frame
    frm_log = ttk.Frame(content, padding=(20, 20, 20, 20 ))
    frm_log.grid(column=0, row=1, sticky=(N,W,S,E))
    frm_log.grid_columnconfigure(1, weight=1)
    frm_log.grid_rowconfigure(2, weight=1)

    # Create and grid the logging listbox frame
    frm_log = ttk.Frame(content, padding=(20, 20, 20, 20 ))
    frm_log.grid(column=0, row=3, columnspan = 2, sticky=(N,W,S,E))
    frm_log.grid_columnconfigure(1, weight=1)
    frm_log.grid_rowconfigure(1, weight=1)

    # create the logging list box, allow multiple items to be selected including use of SHIFT 
    h_log = Listbox(frm_log, height=15, width =132, selectmode = EXTENDED)
    h_log.grid(column=0, row=1, sticky=(N,W,E,S))

    # create the scroll bar and attach it to the list box
    h_scroll_log = ttk.Scrollbar(frm_log, orient=VERTICAL, command=h_log.yview)
    h_scroll_log.grid(column=1, row=1, sticky=(N,W,S))
    h_log['yscrollcommand'] = h_scroll_log.set
    ttk.Sizegrip().grid(column=1, row=1, sticky=(S,E))

    return (frm_log, h_log)
    
def create_command_area():

    # create the main frame
    frm_cmd = ttk.Frame(content, padding=(20, 20, 20, 20 ))
    frm_cmd.grid(column=1, row=1, sticky=(N,W,S,E))
    frm_cmd.grid_columnconfigure(1, weight=1)
    frm_cmd.grid_rowconfigure( 2, weight=1)

    return (frm_cmd)



    
# left mouse clicks on canvas
def cnvs_left_click(event):

    global beacon_index
    global beacon_coord
        
    global work_area_coord
    global veh_pos
    
    # mouse clicks can be related to a range of functions, detremine which function
    # based upon the boolean that was set when the function was requested
    
    
    if b_set_veh_start_pos:
    
        print ('cnvs_left_click: b_set_veh_start_pos clicked at', event.x, event.y)
        
        # clear the X at the last location
        (x_last, y_last) = veh_pos
        
        # overwrite the last location with red rectangle; mouse entered coordinates - no conversion required
        cnvs.create_rectangle(x_last - 10, yc(y_last - 10), x_last + 10, yc(y_last + 10), outline='red', fill='red')
        
        # write a X at the mouse click location; mouse entered coordinates 
        cnvs.create_text(event.x, event.y, text='X', fill = 'black')
        
        # veh_pos is in pixels
        x_veh = event.x
        y_veh = yc(event.y)
        veh_pos = (x_veh, y_veh)

        # convert vehicle position to cm
        x_veh_cm = x_veh * cm_pix
        y_veh_cm = y_veh * cm_pix
        
        # update the numerical vehicle position indication in cm
        sv_veh_pos.set(str(x_veh_cm) + ' , ' + str(y_veh_cm))
        
        
        
        
    elif b_set_work_area:
        
        # add each (x,y) coordinate to the work area list. When the Finish Work Area button is pressed
        # draw the polynom showing the work area
        print ('cnvs_left_click: b_set_work_area clicked at', event.x, event.y)
        
        # write a W at the mouse click location to assist in marking out the work area; mouse entered coordinates - no conversion required
        cnvs.create_text(event.x, event.y, text='W', fill = 'black')
        
        # append to the list
        work_area_coord.append((event.x, event.y))
        
    else:    
        print ('cnvs_left_click: left mouse button clicked')
        

# button 'Set / Finish vehicle start position' is pushed
def set_veh_start_pos():

    global b_set_veh_start_pos
    global f_b01_dist
    global sv_b01_dist
    global b01_entry
    
    # toggle the flag as the event will be handle by left_mouse()
    b_set_veh_start_pos = not b_set_veh_start_pos
    
    # change the button text
    if b_set_veh_start_pos:
        btn_set_veh_start_pos.config(text = 'Finish start positon')
        h_log.insert('end','Set mower start position')
      
    else:
    
        # convert vehicle position to cm
        (x, y) = veh_pos
        x_veh_cm = x * cm_pix
        y_veh_cm = y * cm_pix
        veh_pos_cm = (x_veh_cm,y_veh_cm)
        
        # send a 'vehicle start position' message to the positioning thread
        data = (veh_pos_cm, cm_pix)                        # package is x ccordinate, y coordinate and the scale factor
        que.push_queue(cmd_q_thread0, que.t_vehicle_start_position, data)
            
        # log
        h_log.insert('end','Mower ' + ' set at position (' + str(x_veh_cm) + ',' +  str(y_veh_cm) + ') cm')
        
        # change the button text
        btn_set_veh_start_pos.config(text = 'Set mower start position')
        h_log.insert('end', 'Finished setting mower start position')
    
    
    
# button 'Set / Finish Work Area' is pushed, define a new work area, clear automatically any previous work area first
def set_work_area():
    
    global b_set_work_area
    global work_area_coord
    global last_work_area_coord
    
    # toggle the flag as entering the coordinates will be handle by left_mouse()
    b_set_work_area = not b_set_work_area
    
    # change the button text
    if b_set_work_area:
        
        # first clear the previous work area
        last_work_area_coord = work_area_coord
        
        # start fresh
        work_area_coord = []
    
        # draw a polynom in background color to clear
        if not (last_work_area_coord == []):
            print ('set_work_area: last_work_area_coord =', last_work_area_coord)
            cnvs.create_polygon(last_work_area_coord, outline='red', fill='red', width=3)
  
        # change the function of the button
        btn_set_work_area.config(text = 'Finish Work Area')
        h_log.insert('end','Setting Work Area')
        
        
    else:
    
        # all coordinates entered, finialize the creation of the work area
        
        # first overwrite the W with red rectangle; mouse entered coordinates - no conversion required
        for coord in work_area_coord:
            (x, y) = coord
            cnvs.create_rectangle(x - 10, y - 10, x + 10, y + 10, outline='red', fill='red')

        # draw the polynom representing the work area; mouse entered coordinates - no conversion required
        cnvs.create_polygon(work_area_coord, outline='green', fill='green', width=3)
  
        # now convert the work area to cm and store with lower left origin
        # tba
        
        # change the function of the button
        btn_set_work_area.config(text = 'Set Work Area ')
        h_log.insert('end', 'Finish setting Work Area')

        


def set_position():

    print ('set_position:  clicked')


# start / stop positioning
def toggle_position_thread():

    global t0
    global kill_thread_0
    

    global b_start_pos
    
    # toggle the flag as the event will be handle by left_mouse()
    b_start_pos = not b_start_pos
    
    # change the button text
    if b_start_pos:
        btn_toggle_pos.config(text = 'Stop')
        h_log.insert('end','Starting positioning')
        
        # start the positioning thread
        pos.kill_thread_0 = False
        t0 = threading.Thread(target=pos.get_position, args=(cmd_q_thread0, res_q_thread0))
        t0.start()   
        
    
    else:
        btn_toggle_pos.config(text = 'Start ')
        h_log.insert('end', 'Stopping positioning')
        
        # set the kill signal
        pos.kill_thread_0 = True

        # wait until the thread has stopped
        while True:
            if not t0.is_alive():
                return()
                 




# independent of user generated events, update the HMI every second
def hmi_update():


        global veh_pos
        global veh_color

        
        
        pkgs = []
        que.expire_threshold = 60        # threshold in seconds when queue pkg expire
        
        # the vehicle location in pixels
        last_veh_pos = veh_pos
        (x_veh, y_veh) = veh_pos
        
        # get the position of the vehicle
        pkgs = que.get_queue(res_q_thread0)
        print ('hmi_update: pkgs = ', pkgs)
        
        # unpack the package
        for pkg in pkgs:
            (pkg_time, type, data) = pkg
            now = time.time()
            if now - pkg_time < que.expire_threshold:
            
                # message from positioning thread
                if type == que.t_message:
                
                    h_log.insert('end', data )
                
                # vehicle position update
                elif type == que.t_vehicle_position:
                
                    print ('type 1 pkg received')
                    
                    # the position is received in cm
                    x_veh_cm = int(data[0])         
                    y_veh_cm = int(data[1])

                    # update the numerical vehicle position on the screen
                    sv_veh_pos.set(str(x_veh_cm) + ' , ' + str(y_veh_cm))
                    
                    # convert to pixel
                    x_veh = data[0] / cm_pix         
                    y_veh = data[1] / cm_pix
                    veh_pos = (x_veh, y_veh)

                    print ('hmi_update: new x_veh_cm = ', x_veh_cm, ' new y_veh_cm = ', y_veh_cm, ' cm')

                    
                # beacon position update - only during beacon discovery
                elif type == que.t_beacon_position:

                    beacon_pos = data
                    for bcn in range(0,3):
                    
                        # unpack the beacon position x,y received in cm
                        (x_cm,y_cm) = beacon_pos[bcn]
                        
                        # round to 2 decimal places
                        x_cm = round(x_cm, 2)
                        y_cm = round(y_cm, 2)
                        
                        # update the numerical indication
                        sv_beacon = sv_beacons[bcn]
                        sv_beacon.set(str(x_cm) + ' , ' + str(y_cm))
                
                        # write a B at the mouse click location
                        x_pix = int(x_cm / cm_pix)
                        y_pix = int(y_cm / cm_pix)
                        cnvs.create_text(x_pix, yc(y_pix), text='B' + str(bcn), fill = 'black')
        
            else:
                print ('hmi_update: pkg expired', pkg)

        # now update the canvas which updated every cycle in order to make a blinking vehicle position
        
        # if the vehicle position changed, clear the last position from the screen
        if not (last_veh_pos == veh_pos):
            (x, y) = last_veh_pos
            cnvs.create_text(x, yc(y), text='X', fill = 'red', font = my_font)  

        # write a X at the vehicle location and blink it
        veh_color = 'red' if veh_color == 'black' else 'black'
        cnvs.create_text(x_veh, yc(y_veh), text='X', fill = veh_color) 

        # generate a software event to call this function again after 1 second
        cnvs.after(1000, hmi_update)
    
    

 # y convert - convert canvas origin from upper left corner to lower left corner and vice versa
def yc(y):

    # for (x,y) coordinates, only y is converted:
    # (0,0)         -> (0,p_y_sz)
    # (0,p_y_sz/2)  -> (0,p_y_sz/2)
    # (0,p_y_sz)    -> (0,0)
    
    y = -y + p_y_sz    
    return (y)   

    
    
    
 
#############################################################################   
# main




root = Tk()


# create a bold font to overwrite test that needs to be deleted on the canvas
my_font =  tkinter.font.Font(family='Arial', size = 10, weight = 'bold')

root.grid_columnconfigure(0, weight=1)
root.grid_rowconfigure(0,weight=1)
root.resizable(0,0)                         # don't allow resizing

# display the essentials in the top line
root.wm_title(my_name  +  " version " + str(version))

# Create and grid the inner content frame in the root to achieve easy padding
content = ttk.Frame(root, padding=(20, 20, 20, 20 ))
content.grid(column=0, row=0, sticky=(N,W,S,E))
content.grid_columnconfigure(2, weight=1)              # maximum 2 colums in main window
content.grid_rowconfigure(2, weight=1)                 # maximum 2 rows in main window

# Create and grid the canvas area
(frm_cnvs, cnvs) = create_canvas_area()

# Create and grid the status frame
(frm_sts, btn_set_veh_start_pos, btn_set_work_area, btn_toggle_pos) = create_status_area()

# create and grid the loggin area
(frm_log, h_log) = create_logging_area()

# create and grid the command area
frm_cmd = create_command_area()

# log the default output directory
h_log.insert('end', 'Ready' )

# start the command and result queue for thread 0 (positioning)
cmd_q_thread0 = queue.Queue()                       # command to the positioning thread
res_q_thread0 = queue.Queue()                       # results from the positioning thread

# start the scheduled hmi update (vehicle position etc)
hmi_update()

# wait for user input
root.mainloop()






