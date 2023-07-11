#!/usr/bin/python3

from tkinter import *
from tkinter import ttk
from tkinter import filedialog
import tkinter.font # bold

import cv2
import numpy as np
import math
import time

# thread imports
import threading
import queue

# application imports
import global_definitions as gd
import mowing as mow
import my_queue as que
import my_gpio 
import RPi.GPIO as GPIO


# global variables in mower.py namespace

# canvas size in pixels
cm_pix = 3                                      # scale_factor (cm per pixel)
p_x_sz = 500                                    # canvas size will be 1500 cm
p_y_sz = 500

# beacons
max_no_bcn = 32                                 # maximum number of beacons supported
sv_beacons = []                                 # screen variables for beacons
b_set_veh_start_pos = False                     # state of the set vehicle button
beacon_pos_cm = []                              # (beacon_no,x,y) coordinates of beacons in cm
h_beacon_pos_cm = [None] * max_no_bcn           # handles to maximum 32 beacon positions on canvas
    
# work area
b_set_work_area = False
work_area_coord = []                            # (x,y) coordinates of work area in pixels
work_area_coord_cm = []                         # (x,y) coordinates of work area in cm
last_work_area_coord = []                       # (x,y) coordinates of last work area in pixels

# starting and stopping of threads
b_start_mowing = False                          # state of mowings stop/start button
 
# vehicle
veh_pos = (0,0,-1)                              # vehicle position (x,y, veh_brng) in pixels; (0,0): lower left corner, -1: not know yet
veh_pos_cm = (0,0,-1)                           # vehicle position (x,y) in cm and vehicle bearing
veh_color = 'red'
h_veh_pos = None                                # handle to vehicle object on canvas
sv_veh_pos = None                               # string variable vehicle position

# grid
b_grid = False                                  # state of the grid on the canvas

# tracing
b_trace = False                                 # tracing state



# create the canvas area
def create_canvas_area():
    
    my_name = 'create_canvas_area()'

    global h_veh_pos
    global h_beacon_pos_cm
    
    # create the main frame
    frm_cnvs = ttk.Frame(content, padding=(20, 20, 20, 20 ))
    frm_cnvs.grid(column=0, row=0, rowspan = 2, sticky=(N,W,S,E))
    frm_cnvs.grid_columnconfigure(1, weight=1)
    frm_cnvs.grid_rowconfigure(1, weight=1)

    # create the canvas to draw
    cnvs = Canvas(frm_cnvs, bg="red", height=p_x_sz, width=p_y_sz)
    cnvs.grid()

    # mark the intial positions of the objects on the canvas, this is to streamline
    # later updates that first delete and then recreate the object when it moves
    
    # vehicle position and bearing
    (x,y, veh_brng) = veh_pos
    h_veh_pos = cnvs.create_text(x, yc(y), text='X', fill = 'black') 
    
    # beacon position
    for bcn in range (len(beacon_pos_cm)):
        (bcn_no,x,y) = beacon_pos_cm[bcn]
        h_beacon_pos_cm[bcn] = cnvs.create_text(x, yc(y), text= 'B' + str(bcn_no), fill = 'black')
        
    #print (f'{my_name} h_beacon_pos_cm = {h_beacon_pos_cm}')
    
    # capture the left mouse button when clicked
    cnvs.bind("<Button-1>", cnvs_left_click)   

    return (frm_cnvs, cnvs)

        
        
def create_status_area():

    global sv_beacons
    global sv_veh_pos
    global mow_pattern
    global b_grid               # grid on/off
    global b_trace              # tracing on/off
    
    my_name = 'create_status_area()'
    
    # create the main frame
    frm_sts = ttk.Frame(content, padding=(20, 20, 20, 20 ))
    frm_sts.grid(column=1, row=0, sticky=(N,W,S,E))
    frm_sts.grid_columnconfigure(10, weight=1)
    frm_sts.grid_rowconfigure(20, weight=1)
     
    # create the beacon label
    Label( frm_sts, text='Beacons').grid(column=0, row=0, sticky=(N,W))
    
    # create a seperate frame for the beacons of 4 rows by 8 colums
    frm_bcn = ttk.Frame(frm_sts, padding=(0, 0, 0, 0 ))
    frm_bcn.grid(column=1, row=0, columnspan = 3, sticky=(N,W,S,E))
    frm_bcn.grid_columnconfigure(8, weight=1)
    frm_bcn.grid_rowconfigure(4, weight=1)
    
    # create the beacon coordinates string vars
    t_col = 0
    t_row = 0
    for var in range(max_no_bcn):
 
        sv_beacon = StringVar(value = 'B-- ---- , ----')
        lbl_beacon1 = Label(frm_bcn, textvariable=sv_beacon).grid(column=t_col, row=t_row)
        sv_beacons.append(sv_beacon)
        t_col += 1
        
        # build a matrix of 3 rows and 8 columns
        if t_col > 7:
            t_row += 1
            t_col = 0

    # create the grid label
    Label( frm_sts, text='Grid').grid(column=0, row=4, sticky=(N,W))

    # create the checkbox to switch grid on and off
    chk_grid = ttk.Checkbutton(frm_sts, variable=bv_grid, onvalue=True, offvalue=False, command = toggle_grid)
    chk_grid.grid(column=1, row=4, sticky=(N,W))
    bv_grid.set(False)
    b_grid = False
    
    # create the tracing label
    Label( frm_sts, text='Tracing').grid(column=0, row=5, sticky=(N,W))
    
    # create the checkbox to switch tracing on and off  
    chk_trace = ttk.Checkbutton(frm_sts, variable=bv_trace, onvalue=True, offvalue=False, command = toggle_trace)
    chk_trace.grid(column=1, row=5, sticky=(N,W))
    bv_trace.set(False)  
    b_trace = False
    
    # create the Vehicle position label
    Label( frm_sts, text='Veh Pos').grid(column=0, row=6, sticky=(N,W))

    # create the vehicle position string variable
    sv_veh_pos = StringVar(value = '--- , --- / ---')
    lbl_veh_pos = Label(frm_sts, textvariable=sv_veh_pos).grid(column=1, row=6)
    
    # create the button to get the vehicle position, will be enabled by loading beacons
    btn_get_veh_start_pos = ttk.Button(frm_sts, text ="Get Mower Position", state = DISABLED, command = get_veh_start_pos)
    btn_get_veh_start_pos.grid(column=2, row=6, sticky=(N,W))
    
    # create the button to position the vehicle, will be enabled by loading beacons
    btn_set_veh_start_pos = ttk.Button(frm_sts, text ="Set Mower Start Position", state = DISABLED, command = set_veh_start_pos)
    btn_set_veh_start_pos.grid(column=3, row=6, sticky=(N,W))
    
    # create the work area label
    Label( frm_sts, text='Work Area').grid(column=0, row=7, sticky=(N,W))
    
    # create the button to set the work area
    btn_set_work_area = ttk.Button(frm_sts, text ="Set Work Area", command = set_work_area)
    btn_set_work_area.grid(column=2, row=7, sticky=(N,W))
        
    
    # create the Mowing label
    Label( frm_sts, text='Mowing').grid(column=0, row=9, sticky=(N,W))
    
    # create the radio buttons for mowing patterns
    mow_pattern =  IntVar()                         # not sure if mow_pattern = IntVar() should be moved to main()
    mow_pattern.set(gd.mow_spiral)
    Radiobutton(frm_sts ,text="Traditional",variable=mow_pattern,value=gd.mow_traditional).grid(row = 9,column = 1, sticky=(N,W))
    Radiobutton(frm_sts ,text="Spiral",variable=mow_pattern,value=gd.mow_spiral).grid(row = 10,column = 1, sticky=(N,W))
    Radiobutton(frm_sts ,text="Random",variable=mow_pattern,value=gd.mow_random).grid(row = 11,column = 1, sticky=(N,W))
    
    # create the button to start/stop mowing
    btn_toggle_mow = ttk.Button(frm_sts, text ="Start", command = toggle_mowing_thread)
    btn_toggle_mow.grid(column=2, row=9, sticky=(N,W))
    
    # return all handles that will be used outside of this function
    return (frm_sts, btn_get_veh_start_pos, btn_set_veh_start_pos, btn_set_work_area, btn_toggle_mow)

def create_man_cntrl_area():

    my_name = 'create_man_cntrl_area()'
    
    # create the main frame
    frm_man_ctl = ttk.Frame(content, padding=(20, 20, 20, 20 ))
    frm_man_ctl.grid(column=1, row=1, sticky=(N,W,S,E))
    frm_man_ctl.grid_columnconfigure(3, weight=1)
    frm_man_ctl.grid_rowconfigure(4, weight=1)
    
    # Creating a photoimage object to use image
    a_n = PhotoImage(file = r"./png/arrow_north.png")
    a_e = PhotoImage(file = r"./png/arrow_east.png")
    a_s = PhotoImage(file = r"./png/arrow_south.png")
    a_w = PhotoImage(file = r"./png/arrow_west.png")

    # create the label
    Label( frm_man_ctl, text='Manual Vehicle Control').grid(column=0, row=0, columnspan=3, sticky=(N,E,W))
    
    # Move forward button
    btn_a_n = ttk.Button(frm_man_ctl, text='Forward', image = a_n, command = mv_forwrd)
    btn_a_n.grid(column=1, row=1, sticky=(N))

    # Turn CCW button
    btn_a_w = ttk.Button(frm_man_ctl, text='CCW', image = a_w, command = mv_cntclk)
    btn_a_w.grid(column=0, row=2, sticky=(N))

    # Stop button
    btn_stp = ttk.Button(frm_man_ctl, text='Stop', image = a_s, command = mv_stop)
    btn_stp.grid(column=1, row=2, sticky=(N))
    
    # Turn CW button
    btn_a_e = ttk.Button(frm_man_ctl, text='CW', image = a_e, command = mv_clk)
    btn_a_e.grid(column=2, row=2, sticky=(N))
    
    # Move backward button
    btn_a_s = ttk.Button(frm_man_ctl, text='Backward', image = a_s, command = mv_bckwrd)
    btn_a_s.grid(column=1, row=3, sticky=(N))

    return(frm_man_ctl)


def create_logging_area():

    my_name = 'create_logging_area()'
    
    # create the main frame
    frm_log = ttk.Frame(content, padding=(20, 20, 20, 20 ))
    frm_log.grid(column=0, row=2, sticky=(N,W,S,E))
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
    

def toggle_trace():

    global b_trace
    
    if b_trace:
    
        # delete the tracing
        cnvs.delete("tracing")
        b_trace = False
    else:
        # any tracing will be done under tag='tracing' by function trace_line
        b_trace = True
        
    print (f'{my_name} b_trace = {b_trace}')
    return()
    
    
# tracing is switched on, draw the line the vehicle from last position to current position
def trace_line(last_veh_pos_pix, veh_pos_pix):

    my_name = 'trace_line()'
    
    # the last vehicle position
    (lvpx,lvpy, lvb) = last_veh_pos_pix
    
    # do not trace the very first line from the origin to the start postion
    if not(lvpx == 0 and lvpy == 0):
        
        # convert the y coordinate of both coordinates to match the cartesian coordinate system
        lvp = (lvpx, yc(lvpy))
    
        # the current vehicle position
        (vpx,vpy, vb) = veh_pos_pix
        vp = (vpx, yc(vpy))
        
        # and draw the line, tag it to be able to erase it later 
        cnvs.create_line([lvp, vp], tag='tracing')

    return()
    

    
# toggle the grid on and off
def toggle_grid():

    global b_grid

    my_name = 'toggle_grid()'
    
    grid_size_pix = int(round(150 / cm_pix, 0))        # grid size 
    
    if b_grid:
    
        print (f'{my_name} b_grid = true')
   
        # delete the grid
        cnvs.delete("grid_lines")
        b_grid = False
    
    else:
        # draw the grid
        
        print (f'{my_name} b_grid = false')
        w = cnvs.winfo_width()          # Get current width of canvas
        h = cnvs.winfo_height()         # Get current height of canvas
        cnvs.delete('grid_lines')       # Will only remove the grid_lines

        # Creates all vertical lines at intervals of 100
        for i in range(0, w, grid_size_pix):
            cnvs.create_line([(i, 0), (i, h)], tag='grid_lines')

        # Creates all horizontal lines at intervals of 100
        for i in range(0, h, grid_size_pix):
            cnvs.create_line([(0, i), (w, i)], tag='grid_lines')

        b_grid = True
        
    return()


# open a layout file
def open_layout():

    global work_area_coord_cm
    global work_area_coord
    global beacon_pos_cm
    
    my_name = 'open_layout()'
    
    lines  = []
    fields = []
    
    # present a file open dialog to search the current directory 
    root.filename =  filedialog.askopenfilename(initialdir = ".",title = "Select file",filetypes=[("layout files", ".txt")])
    
    # report to the hmi
    h_log.insert('end', 'Using layout file  ' + root.filename )
    
    # read the file content
    (status,lines) = read_text_file(root.filename)
    for line in lines:
        print (f'{my_name} {line}')
        (fields) = line.split(',')
        
        # beacons: B,beacon_1,beacon_1_x,beacon_1_y,beacon_2,beacon_2_x,beacon_2_y, .. , beacon_n,beacon_n_x,beacon_n_y
        if fields[0] == 'B':
            # a list of (x,y) coordinates of each beacon
            ii = 1
            bcn = 0
            while ii < len(fields):
                beacon_pos_cm.append((int(fields[ii]), int(fields[ii + 1]), int(fields[ii + 2])))
                ii = ii + 3  
                bcn += 1
                
            canvas_beacon_update(beacon_pos_cm)
            
            # log the beacon locations were restored
            h_log.insert('end', 'Restored Beacon locations from lay-out file')
            
            # enable the 'get_vehicle_position' and 'set_vehicle_position' button (Disabled at startup)
            btn_get_veh_start_pos["state"] = "normal"
            btn_set_veh_start_pos["state"] = "normal"
 
        # work area: W, corner_1_x,corner_1_y, corner_2_x,corner_2_y, ..., corner_n_x,corner_n_y
        if fields[0] == 'W':
        
            # first clear the temporary work area markers 
            cnvs.delete("tag_work_area_markers")    
            
            # list of (x,y) coordinates of each the work area in cm
            ii = 1
            while ii < len(fields):
                work_area_coord_cm.append((int(fields[ii]), int(fields[ii + 1])))
                ii = ii + 2       
            print (f'{my_name} work_area_coord_cm = {work_area_coord_cm}')
            
            # now convert the work area from cm to pixel and store with lower left origin
            work_area_coord = []
            for coord in work_area_coord_cm:
                print (f'{my_name} coord = {coord}')
                (x,y) = coord
                
                # change origin to (0,0) and convert to cm
                x_pix = x / cm_pix
                y_pix = yc(y / cm_pix)
                coord_pix = (x_pix, y_pix)
                work_area_coord.append(coord_pix)
            
            # draw the polynom representing the work area
            cnvs.create_polygon(work_area_coord, outline='green', fill='green', width=3, tag='tag_work_area')
            
            # log the work area was restored
            h_log.insert('end', 'Restored Work Area from lay-out file')
 
    return()

    
# open and read a text file
def read_text_file(file_name):
   
    my_name = 'read_text_file()'
    
    lines = []
    
    try:
        with open(file_name) as f:
            for line in f:            
            
                # cleanup the lines and append to list
                lines.append(cleanup_line(line))

            success = 1
    
    except:
        # log the issue in the output window
        h_log.insert('end', 'Cannot open file ' + file_name )
        success = 0
        
    return(success, lines) 


# cleanup a line by removing eol and duplicate white space
def cleanup_line(line):      

    # strip "end of line" from the line
    line = line.rstrip('\n')
    
    # replace duplicate white spaces with a single space
    line = ' '.join(line.split())

    return (line)
    
    

    
# left mouse clicks on canvas
def cnvs_left_click(event):

        
    global work_area_coord
    global veh_pos
    global h_veh_pos
    
    my_name = 'cnvs_left_click()'
    
    # mouse clicks can be related to a range of functions, determine which function
    # based upon the boolean that was set when the function was requested
    
    
    if b_set_veh_start_pos:
    
        print (f'{my_name} b_set_veh_start_pos clicked at {event.x} {event.y}')
        
        # clear the X at the last location
        (x_last, y_last, veh_brng) = veh_pos
        
        # clear the last location of the vehicle
        cnvs.delete(h_veh_pos)
        
        # write a X at the mouse click location; mouse entered coordinates 
        h_veh_pos = cnvs.create_text(event.x, event.y, text='X', fill = 'black')
        
        # veh_pos is in pixels
        x_veh = event.x
        y_veh = yc(event.y)
        veh_pos = (x_veh, y_veh, veh_brng)

        # convert vehicle position to cm
        x_veh_cm = x_veh * cm_pix
        y_veh_cm = y_veh * cm_pix
        
        # update the numerical vehicle position indication in cm
        sv_veh_pos.set(str(x_veh_cm) + ' , ' + str(y_veh_cm) + ' / ' + str(veh_brng))
        
        
        
        
    elif b_set_work_area:
        
        # add each (x,y) coordinate to the work area list. When the Finish Work Area button is pressed
        # draw the polynom showing the work area
        print (f'{my_name} b_set_work_area clicked at {event.x} {event.y}')
        
        # write a W at the mouse click location to assist in marking out the work area; mouse entered coordinates - no conversion required
        cnvs.create_text(event.x, event.y, text='W', fill = 'black', tag="tag_work_area_markers")
        
        # append to the list
        work_area_coord.append((event.x, event.y))
        
    else:    
        print (f'{my_name} left mouse button clicked')


# button 'get vehicle position' is pushed
# this will allow the mower to determine it's own position via trilaterationn
# and the user does not need to enter the start position via the hmi
def get_veh_start_pos():
    
    my_name = 'get_veh_start_pos()'
    print (f'{my_name} get_vehicle_position clicked')        

    # send a 'get vehicle position' message to the mowing thread
    # this will start a triangulation
    data = (beacon_pos_cm)  
    que.push_queue(cmd_q_thread1, que.t_get_position, data)
        

# button 'Set / Finish vehicle start position' is pushed
# if trilateration does not work, the mower position can be set manually
def set_veh_start_pos():

    global b_set_veh_start_pos
    
    my_name = 'set_veh_start_pos()'
    
    # toggle the flag as the event will be handle by left_mouse()
    b_set_veh_start_pos = not b_set_veh_start_pos
    
    # change the button text
    if b_set_veh_start_pos:
        btn_set_veh_start_pos.config(text = 'Finish start positon')
        h_log.insert('end','Set mower start position')
      
    else:
    
        # convert vehicle position to cm, the vehicle bearing is yet unknown 
        (x, y, veh_brng) = veh_pos
        x_veh_cm = x * cm_pix
        y_veh_cm = y * cm_pix
        veh_pos_cm = (x_veh_cm,y_veh_cm, veh_brng)
            
        # log
        h_log.insert('end','Mower ' + ' set at position (' + str(x_veh_cm) + ',' +  str(y_veh_cm) + ') cm - bearing ' + str(veh_brng))
        
        # change the button text
        btn_set_veh_start_pos.config(text = 'Set mower start position')
        h_log.insert('end', 'Finished setting mower start position')
 
 
    
    
# button 'Set / Finish Work Area' is pushed, define a new work area, clear automatically any previous work area first
def set_work_area():
    
    global b_set_work_area
    global work_area_coord
    global work_area_coord_cm
    global last_work_area_coord
    
    my_name = 'set_work_area()'
    
    # toggle the flag as entering the coordinates will be handle by left_mouse()
    b_set_work_area = not b_set_work_area
    
    # change the button text
    if b_set_work_area:
        
        # first clear the previous work area
        last_work_area_coord = work_area_coord
        
        # start fresh
        work_area_coord = []
    
        # clear the work area if it exists by tag
        if not (last_work_area_coord == []):
            print (f'{my_name} last_work_area_coord = {last_work_area_coord}')
            cnvs.delete("tag_work_area")
            
        # change the function of the button
        btn_set_work_area.config(text = 'Finish Work Area')
        h_log.insert('end','Setting Work Area')
        
        
        
    else:
    
        # all coordinates entered, finialize the creation of the work area
        
        # first clear the temporary work area markers 
        cnvs.delete("tag_work_area_markers")    

        # draw the polynom representing the work area; mouse entered coordinates - no conversion required
        cnvs.create_polygon(work_area_coord, outline='green', fill='green', width=3, tag='tag_work_area')
  
        # now convert the work area to cm and store with lower left origin
        work_area_coord_cm = []
        for coord in work_area_coord:
            (x,y) = coord
            
            # change origin to (0,0) and convert to cm
            x_cm = x * cm_pix
            y_cm = yc(y) * cm_pix
            coord_cm = (x_cm, y_cm)
            work_area_coord_cm.append(coord_cm)
        
        # change the function of the button
        btn_set_work_area.config(text = 'Set Work Area ')
        h_log.insert('end', 'Finish setting Work Area')



def set_position():
    
    my_name = 'set_position()'

    print (f'{my_name}  clicked')


# start / stop mowing
def toggle_mowing_thread():

    global t1
    global kill_thread_1
    
    global b_start_mowing
    
    my_name = 'toggle_mowing()'
    
    # toggle the flag as the event will be handle by left_mouse()
    b_start_mowing = not b_start_mowing
    
    # change the button text
    if b_start_mowing:
        btn_toggle_mow.config(text = 'Stop')
        h_log.insert('end','Starting mowing')
        
        # get the mowing pattern
        mowing_pattern = mow_pattern.get()
        
        # the current position is the starting position 
        (x,y, veh_brng) = veh_pos
        x_veh_cm = x * cm_pix         
        y_veh_cm = y * cm_pix
        veh_pos_cm = (x_veh_cm, y_veh_cm, veh_brng)
        
        # sent the mowing information: start position, mower pattern, work area and beacon position
        data = (veh_pos_cm, mowing_pattern, work_area_coord_cm, beacon_pos_cm)                
        que.push_queue(cmd_q_thread1, que.t_start_mowing, data)
        
    
    else:
        btn_toggle_mow.config(text = 'Start ')
        h_log.insert('end', 'Stopping mowing')
        
        # send the stop mowing signal
        data = 0                    
        que.push_queue(cmd_q_thread1, que.t_stop_mowing, data)
        
        

                
# write position of the beacons on the canvas 
def canvas_beacon_update(beacon_pos_cm):

    global h_beacon_pos_cm
    
    for bcn in range(len(beacon_pos_cm)):
    
        # unpack the beacon position x,y received in cm
        (bcn_no, x_cm, y_cm) = beacon_pos_cm[bcn]
        
        # save space, present as integers
        x_cm = int(x_cm)
        y_cm = int(y_cm)
        
        # update the numerical indication
        sv_beacon = sv_beacons[bcn]
        sv_beacon.set(str(bcn_no) + ' ' + str(x_cm) + ',' + str(y_cm) + ' ')

        # delete the previous beacon from the canvas
        cnvs.delete(h_beacon_pos_cm[bcn])
        
        # write a B at the mouse click location
        x_pix = int(x_cm / cm_pix)
        y_pix = int(y_cm / cm_pix)
        h_beacon_pos_cm[bcn] = cnvs.create_text(x_pix, yc(y_pix), text='B' + str(bcn_no), fill = 'black') 
        
    return()


# manual control - move forward pressed
def mv_forwrd():

    my_name = 'mv_forwrd()'
    
    turn_angle = 0
    distance = 50
    
    print (f'{my_name}')
    data = (turn_angle, distance)
    que.push_queue(cmd_q_thread1, que.t_man_mv_forwrd, data)
    return()

# manual control - turn CCW pressed
def mv_cntclk():

    my_name = 'mv_cntclk()'
    
    turn_angle = -30
    distance = 0
    
    print (f'{my_name}')
    data = (turn_angle, distance)
    que.push_queue(cmd_q_thread1, que.t_man_mv_cntclk, data)
    return()

# manual control - turn CW pressed
def mv_clk():

    my_name = 'mv_clk()'
    
    turn_angle = 30
    distance = 0
    
    print (f'{my_name}')
    data = (turn_angle, distance)
    que.push_queue(cmd_q_thread1, que.t_man_mv_clk, data)
    return()
    
# manual control - move backward arrow pressed
def mv_bckwrd():

    my_name = 'mv_bckwrd()'
    
    turn_angle = 0
    distance = -50
    
    print (f'{my_name}')
    data = (turn_angle, distance)
    que.push_queue(cmd_q_thread1, que.t_man_mv_bckwrd, data)
    return()

# manual control - stop pressed
def mv_stop():

    my_name = 'mv_stop()'
    
    distance = 0
    turn_angle = 0
    
    print (f'{my_name}')
    data = (turn_angle, distance)
    que.push_queue(cmd_q_thread1, que.t_man_mv_stop, data)
    return()


# independent of user generated events, update the HMI every second
def hmi_update():


        global veh_color
        global veh_pos
        global h_veh_pos
        global sv_veh_pos
        
        my_name = 'hmi_update()'
        
        pkgs = []
        que.expire_threshold = 60        # threshold in seconds when queue pkg expire
        
        # the vehicle location in pixels
        last_veh_pos = veh_pos
        (x_veh, y_veh, veh_brng) = veh_pos
        
        # service the mowing queue thread 1
        pkgs = que.get_queue(res_q_thread1)
        print (f'{my_name} mowing thread pkgs received = {pkgs}')
        
        # unpack the mowing package
        for pkg in pkgs:
            (pkg_time, ptype, data) = pkg
            now = time.time()
            if now - pkg_time < que.expire_threshold:
 
                # message from positioning thread
                if ptype == que.t_message:
                
                    h_log.insert('end', data )

                # vehicle position fed back from mowing thread
                if ptype == que.t_vehicle_position:
                
                    
                    # the position is received in cm
                    x_veh_cm = int(data[0])         
                    y_veh_cm = int(data[1])
                    veh_brng = int(data[2])

                    # update the numerical vehicle position on the screen
                    sv_veh_pos.set(str(x_veh_cm) + ' , ' + str(y_veh_cm) + ' / ' + str(veh_brng))
                    
                    # convert to pixel
                    x_veh = data[0] / cm_pix         
                    y_veh = data[1] / cm_pix
                    veh_pos = (x_veh, y_veh, veh_brng)

                    # if tracing is switched on, draw the path
                    if b_trace:
                        trace_line(last_veh_pos, veh_pos)
                        
                    print (f'{my_name} x_veh_cm = {x_veh_cm} new y_veh_cm = {y_veh_cm} veh_brng = {veh_brng}')
  

        # now update the canvas which updated every cycle in order to make a blinking vehicle position
        cnvs.delete(h_veh_pos)
            
        # write a X at the vehicle location and blink it
        veh_color = 'red' if veh_color == 'black' else 'black'
        h_veh_pos = cnvs.create_text(x_veh, yc(y_veh), text='X', fill = veh_color) 
        
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



# when the user closes the application, by File > Exit or the X button
def close_app():

    my_name = 'close_app()'
    
    # stop the mowing thread
    gd.kill_thread_1 = True

    # wait until the thread has stopped
    while True:
        if not gd.t1.is_alive():
            break
            
    # release the GPIO; re-initialize to clear all outputs
    setup_GPIO()

    # and exit
    root.destroy()
 
 


# set up the GPIO for all motors
def setup_GPIO():

    my_name = 'setup_GPIO()'

    # setup the GPIO numbering scheme
    GPIO.setmode(GPIO.BCM)
 
    # mirror rotation
    GPIO.setup(my_gpio.MIR_DIR_PIN, GPIO.OUT)
    GPIO.setup(my_gpio.MIR_STEP_PIN, GPIO.OUT)
    GPIO.output(my_gpio.MIR_DIR_PIN, my_gpio.CW)     # start clock wise

    # mirror rotation calibration proximity sensor
    GPIO.setup(my_gpio.MIR_ZERO_PIN, GPIO.IN)     
    
    # drive motor status feedback
    GPIO.setup(my_gpio.WHL_DRV_PIN, GPIO.IN)   
    
    # blade motor control (on or off)
    GPIO.setup(my_gpio.BLD_MOT_PIN, GPIO.OUT)   

    print(f'{my_name} GPIO setup completed')
 


    

 
#############################################################################   
# main

if __name__ == '__main__':
    
    my_name = 'main():'
    
    version = 1.0
    
    # set up the GPIO 
    setup_GPIO()

    root = Tk()

    # when the Windows application Close control (X in top right corner) is used, call the close_app function
    root.protocol("WM_DELETE_WINDOW", close_app)

    # create a bold font to overwrite test that needs to be deleted on the canvas
    my_font =  tkinter.font.Font(family='Arial', size = 10, weight = 'bold')

    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(0,weight=1)
    root.resizable(0,0)                         # don't allow resizing

    # display the essentials in the top line
    root.wm_title(my_name  +  " version " + str(version))

    # set up the File menu 
    menubar = Menu(root)
    filemenu = Menu(menubar, tearoff=0)
    filemenu.add_command(label="Open", command=open_layout)
    filemenu.add_command(label="Exit", command=close_app)
    menubar.add_cascade(label="File", menu=filemenu)
    root.config(menu=menubar)

    # Create and grid the inner content frame in the root to achieve easy padding
    content = ttk.Frame(root, padding=(20, 20, 20, 20 ))
    content.grid(column=0, row=0, sticky=(N,W,S,E))
    content.grid_columnconfigure(2, weight=1)              # maximum 2 colums in main window
    content.grid_rowconfigure(2, weight=1)                 # maximum 2 rows in main window

    # Create and grid the canvas area
    (frm_cnvs, cnvs) = create_canvas_area()


    # Create and grid the status frame
    bv_grid = BooleanVar() 
    bv_trace = BooleanVar() 
    (frm_sts, btn_get_veh_start_pos, btn_set_veh_start_pos, btn_set_work_area, btn_toggle_mow) = create_status_area()

    # create and grid the manual controls area
    (frm_man_ctl) = create_man_cntrl_area()

    # create and grid the loggin area
    (frm_log, h_log) = create_logging_area()

    # log the default output directory
    h_log.insert('end', 'Ready' )

    # start the command and result queue for thread 1 (mowing)
    cmd_q_thread1 = queue.Queue()                       # commands to the mowing thread
    res_q_thread1 = queue.Queue()                       # results from the mowing thread

    # start the mowing thread
    gd.kill_thread_1 = False
    gd.t1 = threading.Thread(target=mow.mowing, args=(cmd_q_thread1, res_q_thread1))
    gd.t1.start() 
            
    # start the scheduled hmi update (vehicle position etc)
    hmi_update()

    # wait for user input
    root.mainloop()






