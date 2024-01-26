#!/usr/bin/env python
# coding: utf-8

"""
######################################################################################################################
# Andrea Favero 20 January 2024
# 
# GUI helping tuninig CUBOTino servos positions.
# This script relates to CUBOTino autonomous, a small and simple Rubik's cube solver robot 3D printed
#
######################################################################################################################
"""


# ################################## Imports  ########################################################################

# python libraries, normally distributed with python
import tkinter as tk                 # GUI library
from tkinter import ttk              # GUI library
import datetime as dt                # date and time library used as timestamp on a few situations (i.e. data log)
import numpy as np                   # array management library
import time                          # import time library  
import glob, os.path, pathlib        # os is imported to ensure the file presence, check/make
import json                          # libraries needed for the json, and parameter import
from getmac import get_mac_address   # library to get the device MAC ddress


# project specific libraries  
from Cubotino_T_pigpiod import pigpiod as pigpiod        # start the pigpiod server
import Cubotino_T_servos as servo    # custom library controlling Cubotino servos and led module
from Cubotino_T_display import display as disp           # custom library controlling Cubotino disply module
from Cubotino_T import tune_image_setup as camera_setup  # import the camera setting up function
from Cubotino_T import read_camera as read_camera        # import the camera reading function
from Cubotino_T import frame_cropping as crop            # import the image cropping function
from Cubotino_T import warp_image as warp                # import the image warping function
from Cubotino_T import close_camera as close_cam         # import the close_camera function
from Cubotino_T import text_font as text_font            # import the font for cv2 function
from get_macs_AF import get_macs_AF                      # import the get_macs_AF function
######################################################################################################################


def reset_camera():
    global expo_shift
    camera_setup(expo_shift, display=disp, gui_debug=False)    # import the camera setting up function

# #################### functions to manage the GUI closing ###########################################################
def on_closing():
    open_top_cover()                             # top_cover is set in its open position
    disp.show_cubotino()                         # cubotino logo is plot to the screen
    print("\nClosing the GUI application\n\n")   # feedback is printed to the terminal
    camWindow.destroy()                          # frame camWindow is destroyied
    servoWindow.destroy()                        # frame servoWindow is destroyied
    servo.servo_off()                            # PWM signal interruption toward the servos (free to be manually moved, servos dependant)
    servo.quit_func()                            # GPIO for PWM set to a fix level, to prevent servos movements after quitting
    close_cam()                                  # close the camera object
    time.sleep(0.5)                              # little delay
    root.destroy()                               # main window is destroyed
######################################################################################################################




# #################### functions to take and show images  ############################################################
def take_image(refresh=5, widgets_freeze=True):
    """funtion showing a PiCamera image after cropping and warping, for tuning purpose."""
    global picamera_initialized, cv2, camera, width, height
    
    if widgets_freeze:
        labels = (crop_label, warp_label, cam_files_label)
        for label in labels:
            disable_widgets(label, relief="sunken")
        root.update()                          # forced a graphic update

    if not picamera_initialized:               # case the picamera_initialized variable is False
        # initial settings: upload cv2, make the camera object, etc
        cv2, camera, width, height = camera_setup(expo_shift, display=disp, gui_debug=False)
        time.sleep(0.5)
        picamera_initialized = True            # picamera_initialized variable is set True
    
    servo.cam_led_On(cam_led_bright)           # top_cover led is activated
    time.sleep(1.2)
    frame, w, h = read_camera()                # video stream and frame dimensions
    servo.cam_led_Off()                        # top_cover led is de-activated
    
    raised_error = False
    try:
        # frame is cropped in order to limit the image area to analyze
        frame2, w2, h2 = crop(frame, width, height, x_l, x_r, y_u, y_b)
        
        # frame is warped to have a top like view toward the top cube face
        frame2, w2, h2 = warp(frame2, w2, h2, warp_fraction, warp_slicing)
        
        cv2.namedWindow('Cropped and warped image')      # create the cube window
        
        for i in range(refresh):
            cv2.moveWindow('Cropped and warped image', 0,40) # move the window to (0,0)
            cv2.imshow('Cropped and warped image', frame2)   # shows the frame
#             cv2.namedWindow('Camera image')                  # create the cube window
#             cv2.moveWindow('Camera image', 0,h2+100)         # move the window right below the modified one
#             cv2.imshow('Camera image', frame)                # shows the frame
            key = cv2.waitKey(100)                           # refresh time is set to 1 second
        
    except:
        print("Error on cropping and/or warping, try less extreme values")
        raised_error = True
        
    if raised_error:
        font, fontScale, fontColor, lineType = text_font()
        cv2.namedWindow('Cropped and warped image')      # create the cube window
        error_frame = np.zeros([h, w, 3],dtype=np.uint8) # empty array
        error_frame.fill(230)                            # array is filled with light gray
        cv2.putText(error_frame, 'ERROR ON CROPPING', (20, 40), font, fontScale*1.2,(0,0,0),lineType)
        cv2.putText(error_frame, 'AND / OR WARPING', (20, 80), font, fontScale*1.2,(0,0,0),lineType)
        cv2.putText(error_frame, 'TRY LESS EXTREME VALUES', (20, 160), font, fontScale*1.2,(0,0,0),lineType)
        for i in range(refresh):
            cv2.moveWindow('Cropped and warped image', 0,40)  # move the window to (0,0)
            cv2.imshow('Cropped and warped image', error_frame)   # shows the frame
            key = cv2.waitKey(100)                            # refresh time is set to 1 second

        
        
    
    if widgets_freeze:
        for label in labels:
            enable_widgets(label, relief="raised")   # widgets are enabled
        root.update()                                # forced a graphic update
######################################################################################################################




# #################### functions to manage servos settings from/to files   ###########################################

def get_fname_AF(fname, mac_pos):
    """generates a filename based on fname and pos value.
        This is used to match AF specific setting files to the mac address and its position on macs_AF.txt"""
    
    return fname[:-4]+'_AF'+str(mac_pos+1)+'.txt'



def read_servo_settings_file(fname=''):
    """ Function to assign the servo settings to a dictionary, after testing their data type."""   
    
    import os.path, pathlib, json                                    # libraries needed for the json, and parameter import
    # convenient choice for Andrea Favero, to upload the settings fitting his robot based on mac address check                
    from getmac import get_mac_address                               # library to get the device MAC ddress
    
    if len(fname) == 0:                                              # case fname equals empty string
        fname = 'Cubotino_T_servo_settings.txt'                      # fname for the text file to retrieve servos settings
        if eth_mac in macs_AF:                                       # case the script is running on AF (Andrea Favero) robot
            fname = get_fname_AF(fname, mac_pos)                     # generates the AF filename
        else:                                                        # case the script is not running on AF (Andrea Favero) robot
            fname = os.path.join(folder, fname)                      # folder and file name for the servos settings, to be tuned
    
    srv_settings = {}                                                # empty dict to store the settings, or to return empty in case of errors
    if os.path.exists(fname):                                        # case the servo_settings file exists
        with open(fname, "r") as f:                                  # servo_settings file is opened in reading mode
            srv_settings = json.load(f)                              # json file is parsed to a local dict variable
        
        try:                                                         # tentative approach
            # dict with all the servos settings from the txt file
            # from the dict obtained via json.load the srv_settings dict adds individual data type check
            srv_settings['t_min_pulse_width'] = float(srv_settings['t_min_pulse_width'])   # defines the min Pulse With the top servo reacts to
            srv_settings['t_max_pulse_width'] = float(srv_settings['t_max_pulse_width'])   # defines the max Pulse With the top servo reacts to
            srv_settings['t_servo_close'] = float(srv_settings['t_servo_close'])           # Top_cover close position, in gpiozero format
            srv_settings['t_servo_open'] = float(srv_settings['t_servo_open'])             # Top_cover open position, in gpiozero format
            srv_settings['t_servo_read'] = float(srv_settings['t_servo_read'])             # Top_cover camera read position, in gpiozero format
            srv_settings['t_servo_flip'] = float(srv_settings['t_servo_flip'])             # Top_cover flip position, in gpiozero format
            srv_settings['t_servo_rel_delta'] = float(srv_settings['t_servo_rel_delta'])   # Top_cover release angle movement from the close position to release tension
            srv_settings['t_flip_to_close_time'] = float(srv_settings['t_flip_to_close_time'])  # time for Top_cover from flip to close position
            srv_settings['t_close_to_flip_time'] = float(srv_settings['t_close_to_flip_time'])  # time for Top_cover from close to flip position 
            srv_settings['t_flip_open_time'] = float(srv_settings['t_flip_open_time'])     # time for Top_cover from open to flip position, and viceversa
            srv_settings['t_open_close_time'] = float(srv_settings['t_open_close_time'])   # time for Top_cover from open to close position, and viceversa
            srv_settings['t_rel_time'] = float(srv_settings['t_rel_time'])                 # time for Top_cover to release tension from close position
    
            srv_settings['b_min_pulse_width'] = float(srv_settings['b_min_pulse_width'])   # defines the min Pulse With the bottom servo reacts to
            srv_settings['b_max_pulse_width'] = float(srv_settings['b_max_pulse_width'])   # defines the max Pulse With the bottom servo reacts to
            srv_settings['b_servo_CCW'] = float(srv_settings['b_servo_CCW'])               # Cube_holder max CCW angle position
            srv_settings['b_servo_CW'] = float(srv_settings['b_servo_CW'])                 # Cube_holder max CW angle position
            srv_settings['b_home'] = float(srv_settings['b_home'])                         # Cube_holder home angle position
            srv_settings['b_rel_CCW'] = float(srv_settings['b_rel_CCW'])                   # Cube_holder release angle at CCW angle positions, to release tension
            srv_settings['b_rel_CW'] = float(srv_settings['b_rel_CW'])                     # Cube_holder release angle at CW angle positions, to release tension
            srv_settings['b_extra_home_CW'] = float(srv_settings['b_extra_home_CW'])       # Cube_holder release angle from CW to home positions to release tension
            srv_settings['b_extra_home_CCW'] = float(srv_settings['b_extra_home_CCW'])     # Cube_holder release angle from CCW to home positions to release tension
            srv_settings['b_spin_time'] = float(srv_settings['b_spin_time'])               # time for Cube_holder to spin 90 deg (cune not contrained)
            srv_settings['b_rotate_time'] = float(srv_settings['b_rotate_time'])           # time for Cube_holder to rotate 90 deg (cube constrained)
            srv_settings['b_rel_time'] = float(srv_settings['b_rel_time'])                 # time for Cube_holder to release tension at home, CCW and CW positions
            
            return srv_settings                                                           # return settings
            
        except:   # exception will be raised if json keys differs, or parameters cannot be converted to float
            print(f'Error on converting to proper format the servos imported parameters, from {fname}')   # feedback is printed to the terminal
            return {}                            # return (empty) settings
        
    
    else:                                        # case the settings file does not exists, or name differs
        print(f'Could not find {fname}')         # feedback is printed to the terminal
 
    
    





def upload_servo_settings(srv_settings):
    """ Function to assign the servo settings from the dictionary to individual global variables.""" 
    
    global t_min_pulse_width, t_max_pulse_width, t_servo_close, t_servo_open
    global t_servo_read, t_servo_flip, t_servo_rel_delta
    global t_flip_to_close_time, t_close_to_flip_time, t_flip_open_time, t_open_close_time, t_rel_time
    
    global b_min_pulse_width, b_max_pulse_width, b_servo_CCW, b_servo_CW, b_home
    global b_rel_CCW, b_rel_CW, b_extra_home_CW, b_extra_home_CCW
    global b_spin_time, b_rotate_time, b_rel_time


    t_min_pulse_width = srv_settings['t_min_pulse_width']        # defines the min Pulse With the top servo reacts to
    t_max_pulse_width = srv_settings['t_max_pulse_width']        # defines the max Pulse With the top servo reacts to
    t_servo_close = srv_settings['t_servo_close']                # Top_cover close position, in gpiozero format
    t_servo_open = srv_settings['t_servo_open']                  # Top_cover open position, in gpiozero format
    t_servo_read = srv_settings['t_servo_read']                  # Top_cover camera read position, in gpiozero format
    t_servo_flip = srv_settings['t_servo_flip']                  # Top_cover flip position, in gpiozero format
    t_servo_rel_delta = srv_settings['t_servo_rel_delta']        # Top_cover release angle movement from the close position to release tension
    t_flip_to_close_time = srv_settings['t_flip_to_close_time']  # time for Top_cover from flip to close position
    t_close_to_flip_time = srv_settings['t_close_to_flip_time']  # time for Top_cover from close to flip position 
    t_flip_open_time = srv_settings['t_flip_open_time']          # time for Top_cover from open to flip position, and viceversa  
    t_open_close_time = srv_settings['t_open_close_time']        # time for Top_cover from open to close position, and viceversa
    t_rel_time = srv_settings['t_rel_time']                      # time for Top_cover to release tension from close position

    b_min_pulse_width = srv_settings['b_min_pulse_width']        # defines the min Pulse With the bottom servo reacts to
    b_max_pulse_width = srv_settings['b_max_pulse_width']        # defines the max Pulse With the bottom servo reacts to
    b_servo_CCW = srv_settings['b_servo_CCW']                    # Cube_holder max CCW angle position
    b_servo_CW = srv_settings['b_servo_CW']                      # Cube_holder max CW angle position
    b_home = srv_settings['b_home']                              # Cube_holder home angle position
    b_rel_CCW = srv_settings['b_rel_CCW']                        # Cube_holder release angle from CCW angle positions, to release tension
    b_rel_CW = srv_settings['b_rel_CW']                          # Cube_holder release angle from CW angle positions, to release tension
    b_extra_home_CW = srv_settings['b_extra_home_CW']            # Cube_holder release angle from home angle positions, to release tension
    b_extra_home_CCW = srv_settings['b_extra_home_CCW']          # Cube_holder release angle from home angle positions, to release tension
    b_spin_time = srv_settings['b_spin_time']                    # time for Cube_holder to spin 90 deg (cune not contrained)
    b_rotate_time = srv_settings['b_rotate_time']                # time for Cube_holder to rotate 90 deg (cube constrained)
    b_rel_time = srv_settings['b_rel_time']                      # time for Cube_holder to release tension at home, CCW and CW positions




def update_servo_settings_dict():
    """ Function to update the servo settings dictionary based on the global variables.""" 
    
    srv_settings['t_min_pulse_width'] = t_min_pulse_width        # defines the min Pulse With the top servo reacts to
    srv_settings['t_max_pulse_width'] = t_max_pulse_width        # defines the max Pulse With the top servo reacts to
    srv_settings['t_servo_close'] = t_servo_close                # Top_cover close position, in gpiozero format
    srv_settings['t_servo_open'] = t_servo_open                  # Top_cover open position, in gpiozero format
    srv_settings['t_servo_read'] = t_servo_read                  # Top_cover camera read position, in gpiozero format
    srv_settings['t_servo_flip'] = t_servo_flip                  # Top_cover flip position, in gpiozero format
    srv_settings['t_servo_rel_delta'] = t_servo_rel_delta        # Top_cover release angle movement from the close position to release tension
    srv_settings['t_flip_to_close_time'] = t_flip_to_close_time  # time for Top_cover from flip to close position
    srv_settings['t_close_to_flip_time'] = t_close_to_flip_time  # time for Top_cover from close to flip position 
    srv_settings['t_flip_open_time'] = t_flip_open_time          # time for Top_cover from open to flip position, and viceversa  
    srv_settings['t_open_close_time'] = t_open_close_time        # time for Top_cover from open to close position, and viceversa
    srv_settings['t_rel_time'] = t_rel_time                      # time for Top_cover to release tension from close to close position
    
    srv_settings['b_min_pulse_width'] = b_min_pulse_width        # defines the min Pulse With the bottom servo reacts to
    srv_settings['b_max_pulse_width'] = b_max_pulse_width        # defines the max Pulse With the bottom servo reacts to
    srv_settings['b_servo_CCW'] = b_servo_CCW                    # Cube_holder max CCW angle position
    srv_settings['b_servo_CW'] = b_servo_CW                      # Cube_holder max CW angle position
    srv_settings['b_home'] = b_home                              # Cube_holder home angle position
    srv_settings['b_rel_CCW'] = b_rel_CCW                        # Cube_holder release angle from CCW angle positions, to release tension
    srv_settings['b_rel_CW'] = b_rel_CW                          # Cube_holder release angle from CW angle positions, to release tension
    srv_settings['b_extra_home_CW'] = b_extra_home_CW            # Cube_holder release angle from home angle positions, to release tension
    srv_settings['b_extra_home_CCW'] = b_extra_home_CCW          # Cube_holder release angle from home angle positions, to release tension
    srv_settings['b_spin_time'] = b_spin_time                    # time for Cube_holder to spin 90 deg (cune not contrained)
    srv_settings['b_rotate_time'] = b_rotate_time                # time for Cube_holder to rotate 90 deg (cube constrained)
    srv_settings['b_rel_time'] = b_rel_time                      # time for Cube_holder to release tension at home, CCW and CW positions
    
    return srv_settings





def load_previous_servo_settings():
    """ Function load the servo settings from latest json backup file saved."""
    
    fname = 'Cubotino_T_servo_settings.txt'                    # fname for the text file to retrieve settings
    if eth_mac in macs_AF:                                     # case the script is running on AF (Andrea Favero) robot
        fname = get_fname_AF(fname, mac_pos)                   # generates the AF filename
    else:                                                      # case the script is not running on AF (Andrea Favero) robot
        fname = os.path.join(folder, fname)                    # folder and file name for the settings, to be tuned
    
    backup_fname = fname[:-4] + '_backup*.txt'                 # backup filename is made
    backup_files = sorted(glob.iglob(backup_fname), key=os.path.getmtime, reverse=True) # ordered list of backuped settings files 
    if len(backup_files) > 0:                                  # case there are backup setting files
        if len(backup_files) > 10:                             # case there are more than 10 backup files
            while len(backup_files) > 10:                      # iteration until there will only be 10 backup files
                os.remove(backup_files[-1])                    # the oldes backup file is deleted
                backup_files = sorted(glob.iglob(backup_fname), key=os.path.getctime, reverse=True) # ordered list of backuped settings files     
        latest_backup = backup_files[0]                        # latest settings files backed up
        srv_settings = read_servo_settings_file(latest_backup) # settings are read from the latest_backup file
        print(f"\nUploaded settings from latest_backup:  {latest_backup}")  # feedback is printed to the terminal
    else:                                                      # case there are backup setting files
        srv_settings = read_servo_settings_file(fname)         # settings are read from the fname file
        print(f"\nNot found backup files, uploaded settings from file: {fname}")
    
    upload_servo_settings(srv_settings)                        # settings are uploaded (to global variables)
    update_servo_sliders()                                     # sliders are updated





def save_new_servo_settings():
    """ Function to write the servo settings dictionary to json file.
        Before overwriting the file, a backup copy is made.""" 
    
    global srv_settings
    
    fname = 'Cubotino_T_servo_settings.txt'                    # fname for the text file to retrieve settings 
    if eth_mac in macs_AF:                                     # case the script is running on AF (Andrea Favero) robot
        fname = get_fname_AF(fname, mac_pos)                   # generates the AF filename
    else:                                                      # case the script is not running on AF (Andrea Favero) robot
        fname = os.path.join(folder, fname)                    # folder and file name for the settings, to be tuned
    
    if os.path.exists(fname):                                  # case the servo_settings file exists    
        datetime = dt.datetime.now().strftime('%Y%m%d_%H%M%S') # date_time variable is assigned, for file name
        backup_fname = fname[:-4] + '_backup_' + datetime + '.txt' # backup file name
        with open(backup_fname, "w") as f:                     # original servo_settings are saved to a backup file with datetime ref
            f.write(json.dumps(srv_settings, indent=0))        # content of the setting file is saved
            print("\nSaving previous settings to backup file:", backup_fname)
        
        backup_fname = fname[:-4] + '_backup*.txt'             # common name prefix for the backup files
        backup_files = sorted(glob.iglob(backup_fname), key=os.path.getmtime, reverse=True) # ordered list of backuped settings files 
        if len(backup_files) > 10:                             # case there are more than 10 backup files
            while len(backup_files) > 10:                      # iteration until there will only be 10 backup files
                os.remove(backup_files[-1])                    # the oldes backup file is deleted
                backup_files = sorted(glob.iglob(backup_fname), key=os.path.getctime, reverse=True) # ordered list of backuped settings files 
        
        srv_settings = update_servo_settings_dict()            # settings updated to sliders values
        with open(fname, "w") as f:                            # servo_settings file is opened in reading mode
            f.write(json.dumps(srv_settings, indent=0))        # content of the setting file is saved
            print("Saving settings to:", fname)                # feedback is printed to the terminal
    else:                                                      # case the servo_settings file exists
        print(f"File name {fname} does not exists at save_new_servo_settings function")





def update_servo_sliders():
    """function to update the sliders according to the (global) variable."""
    
    # t_min_pulse_width
    # t_max_pulse_width
    s_top_srv_close.set(t_servo_close)
    s_top_srv_rel_delta.set(t_servo_rel_delta)
    s_top_srv_open.set(t_servo_open)
    s_top_srv_read.set(t_servo_read)
    s_top_srv_flip.set(t_servo_flip)
    s_top_srv_flip_to_close_time.set(t_flip_to_close_time)
    s_top_srv_close_to_flip_time.set(t_close_to_flip_time)
    s_top_srv_flip_open_time.set(t_flip_open_time)
    s_top_srv_open_close_time.set(t_open_close_time)
    s_top_srv_rel_time.set(t_rel_time)
    
    s_btn_srv_min_pulse.set(b_min_pulse_width)
    s_btn_srv_max_pulse.set(b_max_pulse_width)
    s_btn_srv_CCW.set(b_servo_CCW)
    s_btn_srv_home.set(b_home)
    s_btn_srv_CW.set(b_servo_CW)
    s_btn_srv_rel_CCW.set(b_rel_CCW)
    s_btn_srv_rel_CW.set(b_rel_CW)
    s_extra_home_CW.set(b_extra_home_CW)
    s_extra_home_CCW.set(b_extra_home_CCW)
    s_btn_srv_spin_time.set(b_spin_time)
    s_btn_srv_rotate_time.set(b_rotate_time)
    s_btn_srv_rel_time.set(b_rel_time)





def last_btn_srv_pos():
    """keep track of the last bottom servo position."""
    global b_servo_pos
    
    if b_servo_pos == 'CCW':
        return b_servo_CCW
    elif b_servo_pos == 'CW':
        return b_servo_CW
    else:
        return b_home
######################################################################################################################





# #################### functions to manage camera settings from/to files   ###########################################
def read_cam_settings_file(fname=''):
    """ Function to assign the cam settings to a dictionary, after testing their data type."""   
    
    import os.path, pathlib, json                              # libraries needed for the json, and parameter import
    # convenient choice for Andrea Favero, to upload the settings fitting his robot based on mac address check                
    from getmac import get_mac_address                         # library to get the device MAC ddress
    
    if len(fname) == 0:                                        # case fname equals empty string
        fname = 'Cubotino_T_settings.txt'                      # fname for the text file to retrieve settings
        if eth_mac in macs_AF:                                 # case the script is running on AF (Andrea Favero) robot
            fname = get_fname_AF(fname, mac_pos)               # generates the AF filename
        else:                                                  # case the script is not running on AF (Andrea Favero) robot
            fname = os.path.join(folder, fname)                # folder and file name for the settings, to be tuned
    
    if os.path.exists(fname):                                  # case the settings file exists
        with open(fname, "r") as f:                            # settings file is opened in reading mode
            cam_settings = json.load(f)                        # json file is parsed to a local dict variable
        
        # update key-values parameters, needed in case of additional parameters added at remote repo
        cam_settings = update_settings_file(fname, settings)
        
        try:           
            # from the dict obtained via json.load the settings dict adds individual data type check
            cam_settings['x_l'] = int(cam_settings['x_l'])         # pixels to remove at the left image side
            cam_settings['x_r'] = int(cam_settings['x_r'])         # pixels to remove at the right image side
            cam_settings['y_u'] = int(cam_settings['y_u'])         # pixels to remove at the upper image side
            cam_settings['y_b'] = int(cam_settings['y_b'])         # pixels to remove at the bottom image side
            cam_settings['warp_fraction'] = float(cam_settings['warp_fraction'])  # warp correction factor
            cam_settings['warp_slicing'] = float(cam_settings['warp_slicing'])    # image slicing after warping
            cam_settings['cam_led_bright'] = float(cam_settings['cam_led_bright'])    # PWM level for the 3W led at Top_cover
            cam_settings['expo_shift'] = float(cam_settings['expo_shift'])    # exposure shift for PiCamera 
            
            return cam_settings
            
        except:   # exception will be raised if json keys differs, or parameters cannot be converted to float
            print(f'Error on converting to proper format the cam imported parameters, from {fname}')   # feedback is printed to the terminal                                  
            return {}                                    # return (empty) settings
    
    else:                                                # case the cam_settings file does not exists, or name differs
        print(f'Could not find {fname}')                 # feedback is printed to the terminal                                  
        return cam_settings                              # return settings




def update_settings_file(fname, settings):
    """Function to check if the existing fname (Cubotino_T_settings.txt) has all the parameters that were instroduced after firt release.
        This will allow every Makers to 'git pull' the updates while pre-serving personal settings (added *_settings.txt to gitignore).
        Cubotino_T.py and Cubotino_T_servos_GUI.py will update Cubotino_T_settings.txt with eventually missed parameters
        and related default values.
        """

    settings_keys = settings.keys()
    any_change = False
    
    if 'frameless_cube' not in settings_keys:
        settings['frameless_cube']='false'
        any_change = True
        
    if 'cover_self_close' not in settings_keys:
        settings['cover_self_close']='false'
        any_change = True
        
    if 's_mode' not in settings_keys:
        settings['s_mode']='7'
        any_change = True
        
    if 'vnc_delay' not in settings_keys:
        settings['vnc_delay']= '0.5'
        any_change = True
    
    if 'built_by' not in settings_keys:
        settings['built_by']=''
        any_change = True
        
    if 'built_by_x' not in settings_keys:
        settings['built_by_x']='25'
        any_change = True
        
    if 'built_by_fs' not in settings_keys:
        settings['built_by_fs']='16'
        any_change = True
    
    if 'expo_shift' not in settings_keys:
        settings['expo_shift']='-1.0'        # this parameter is ignored on OS10 (Buster)
        any_change = True
     
    if any_change:
        print('\nOne time action: Adding new parameters to the Cubotino_T_settings.txt')
        print('Action necessary for compatibility with the latest downloaded Cubotino_T.py \n')
        with open(fname, 'w') as f:
            f.write(json.dumps(settings, indent=0))   # content of the updated setting is saved
    
    return settings






def upload_cam_settings(cam_settings):
    """ Function to assign the cam settings from the dictionary to individual global variables.""" 
    
    global x_l, x_r, y_u, y_b, warp_fraction, warp_slicing, cam_led_bright, expo_shift

    x_l = cam_settings['x_l']                            # pixels to remove at the left image side
    x_r = cam_settings['x_r']                            # pixels to remove at the right image side
    y_u = cam_settings['y_u']                            # pixels to remove at the upper image side
    y_b = cam_settings['y_b']                            # pixels to remove at the bottom image side
    warp_fraction = cam_settings['warp_fraction']        # image warping factor
    warp_slicing = cam_settings['warp_slicing']          # image slicing after warping
    cam_led_bright = cam_settings['cam_led_bright']      # PWM level for the 3W led at Top_cover
    expo_shift = cam_settings['expo_shift']              # exposure shift for PiCamera 




def load_previous_cam_settings():
    """ Function load the cam settings from latest json backup file saved.""" 
    
    fname = 'Cubotino_T_settings.txt'                          # fname for the text file to retrieve settings
    if eth_mac in macs_AF:                                     # case the script is running on AF (Andrea Favero) robot
        fname = get_fname_AF(fname, mac_pos)                   # generates the AF filename
    else:                                                      # case the script is not running on AF (Andrea Favero) robot
        fname = os.path.join(folder, fname)                    # folder and file name for the settings, to be tuned
    
    backup_fname = fname[:-4] + '_backup*.txt'                 # backup filename is made
    backup_files = sorted(glob.iglob(backup_fname), key=os.path.getmtime, reverse=True) # ordered list of backuped settings files 
    if len(backup_files) > 0:                                  # case there are backup setting files
        if len(backup_files) > 10:                             # case there are more than 10 backup files
            while len(backup_files) > 10:                      # iteration until there will only be 10 backup files
                os.remove(backup_files[-1])                    # the oldes backup file is deleted
                backup_files = sorted(glob.iglob(backup_fname), key=os.path.getctime, reverse=True) # ordered list of backuped settings files     
        latest_backup = backup_files[0]                        # latest settings files backed up
        cam_settings = read_cam_settings_file(latest_backup)   # settings are read from the latest_backup file
        print(f"\nUploaded cam settings from latest_backup:  {latest_backup}")  # feedback is printed to the terminal
    else:                                                      # case there are backup setting files
        cam_settings = read_cam_settings_file(fname)           # settings are read from the fname file
        print(f"\nNot found backup files, uploaded settings from file: {fname}")
    
    upload_cam_settings(cam_settings)                          # cam settings are uploaded (to global variables)
    update_cam_sliders()                                       # cam sliders are updated





def update_cam_settings_dict():
    """ Function to update the cam settings dictionary based on the global variables.""" 
    
    cam_settings['x_l'] = x_l                            # pixels to remove at the left image side
    cam_settings['x_r'] = x_r                            # pixels to remove at the right image side
    cam_settings['y_u'] = y_u                            # pixels to remove at the upper image side
    cam_settings['y_b'] = y_b                            # pixels to remove at the bottom image side
    cam_settings['warp_fraction'] = warp_fraction        # image warping factor
    cam_settings['warp_slicing'] = warp_slicing          # image slicing after warping
    cam_settings['cam_led_bright'] = cam_led_bright      # PWM level for the 3W led at Top_cover
    cam_settings['expo_shift'] = expo_shift              # exposure shift for PiCamera 
    
    return cam_settings





def save_new_cam_settings():
    """ Function to write the cam settings dictionary to json file.
        Before overwriting the file, a backup copy is made.""" 
    
    global cam_settings
    
    fname = 'Cubotino_T_settings.txt'                              # fname for the text file to retrieve settings
    if eth_mac in macs_AF:                                         # case the script is running on AF (Andrea Favero) robot
        fname = get_fname_AF(fname, mac_pos)                       # generates the AF filename
    else:                                                          # case the script is not running on AF (Andrea Favero) robot
        fname = os.path.join(folder, fname)                        # folder and file name for the settings, to be tuned
    
    if os.path.exists(fname):                                      # case the servo_settings file exists
        datetime = dt.datetime.now().strftime('%Y%m%d_%H%M%S')     # date_time variable is assigned, for file name
        backup_fname = fname[:-4] + '_backup_' + datetime + '.txt' # backup file name
        with open(backup_fname, "w") as f:                         # original servo_settings are saved to a backup file with datetime ref
            f.write(json.dumps(cam_settings, indent=0))            # content of the setting file is saved
            print("\nSaving previous settings to backup file:", backup_fname)
        
        backup_fname = fname[:-4] + '_backup*.txt'                 # common name prefix for the backup files
        backup_files = sorted(glob.iglob(backup_fname), key=os.path.getmtime, reverse=True) # ordered list of backuped settings files 
        if len(backup_files) > 10:                                 # case there are more than 10 backup files
            while len(backup_files) > 10:                          # iteration until there will only be 10 backup files
                os.remove(backup_files[-1])                        # the oldes backup file is deleted
                backup_files = sorted(glob.iglob(backup_fname), key=os.path.getctime, reverse=True) # ordered list of backuped settings files 
        
        cam_settings = update_cam_settings_dict()                  # settings updated to sliders values
        with open(fname, "w") as f:                                # robot setting file is opened in reading mode
            f.write(json.dumps(cam_settings, indent=0))            # content of the setting file is saved
            print("Saving settings to:", fname)                    # feedback is printed to the terminal
    else:                                                          # case the servo_settings file exists
        print(f"File name {fname} does not exists at save_new_cam_settings function")





def update_cam_sliders():
    """function to update the cam sliders according to the (global) variable."""
    
    s_crop_l.set(x_l)
    s_crop_r.set(x_r)
    s_crop_u.set(y_u)
    s_crop_b.set(y_b)
    s_warp_f.set(warp_fraction)
    s_warp_s.set(warp_slicing)
    s_led_pwm.set(cam_led_bright)
    s_expo_shift.set(expo_shift)

######################################################################################################################







# ################################### functions to get the slider values  ############################################
def servo_close(val):
    global t_servo_close, t_servo_rel_delta, t_servo_pos
    t_servo_close = round(float(s_top_srv_close.get()),3)        # top servo pos to constrain the top cover on cube mid and top layer
    t_servo_rel = round(t_servo_close - float(s_top_srv_rel_delta.get()),3)
    disp.show_on_display('t_srv CLOSE', str(t_servo_close), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
    servo.servo_to_pos('top', t_servo_close)      # top servo is positioned to the slider value
    time.sleep(t_open_close_time)
    servo.servo_to_pos('top', t_servo_rel)        # top servo is positioned to the slider value
    time.sleep(t_rel_time)
    disp.show_on_display('t_srv CLOSE', f'{t_servo_close} ({t_servo_rel})', fs1=19, y2=75, fs2=18)  # feedback is printed to the display
    t_servo_pos = 'close'                         # string variable to track the last top_cover position
    btm_srv_widgets_status()                      # updates the bottom servo related widgests status, based on top servo pos


def servo_release(val):
    global t_servo_rel_delta, t_servo_pos
    t_servo_rel_delta = round(float(s_top_srv_rel_delta.get()),3)  # top servo release position after closing toward the cube
    disp.show_on_display('t_srv RELEASE', f'({t_servo_rel_delta})', fs1=16, y2=75, fs2=18)  # feedback is printed to the display
    t_servo_pos = 'close'                         # string variable to track the last top_cover position
    btm_srv_widgets_status()                      # updates the bottom servo related widgests status, based on top servo pos


def servo_open(val):
    global t_servo_open, t_servo_pos
    t_servo_open = round(float(s_top_srv_open.get()),3)  # top servo pos to free up the top cover from the cube
    disp.show_on_display('t_srv OPEN', str(t_servo_open), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
    servo.servo_to_pos('top', t_servo_open)       # top servo is positioned to the slider value
    t_servo_pos = 'open'                          # string variable to track the last top_cover position
    btm_srv_widgets_status()                      # updates the bottom servo related widgests status, based on top servo pos
    

def servo_read(val):
    global t_servo_read, t_servo_pos
    t_servo_read = round(float(s_top_srv_read.get()),3)  # top servo pos for camera reading
    disp.show_on_display('t_srv READ', str(t_servo_read), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
    servo.servo_to_pos('top', t_servo_read)       # top servo is positioned to the slider value
    t_servo_pos = 'read'                          # string variable to track the last top_cover position
    btm_srv_widgets_status()                      # updates the bottom servo related widgests status, based on top servo pos


def servo_flip(val):
    global t_servo_flip, t_servo_pos
    t_servo_flip = round(float(s_top_srv_flip.get()),3)  # top servo pos to flip the cube on one of its horizontal axis
    disp.show_on_display('t_srv FLIP', str(t_servo_flip), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
    servo.servo_to_pos('top', t_servo_flip)       # top servo is positioned to the slider value
    t_servo_pos = 'flip'                          # string variable to track the last top_cover position
    btm_srv_widgets_status()                      # updates the bottom servo related widgests status, based on top servo pos
    
    
def flip_to_close_time(val):
    global t_flip_to_close_time
    t_flip_to_close_time = round(float(s_top_srv_flip_to_close_time.get()),3)  # time to lower the cover/flipper from flip to close position


def servo_rel_close(val):
    global t_rel_time
    t_rel_time = round(float(s_top_srv_rel_time.get()),3)   # time to release tension at top_cover close position

    
def close_to_flip_time(val):
    global t_close_to_flip_time
    t_close_to_flip_time = round(float(s_top_srv_close_to_flip_time.get()),3)  # time to raise the cover/flipper from close to flip position


def flip_open_time(val):
    global t_flip_open_time
    t_flip_open_time = round(float(s_top_srv_flip_open_time.get()),3)    # time to raise/lower the flipper between open and flip positions


def open_close_time(val):
    global t_open_close_time
    t_open_close_time = round(float(s_top_srv_open_close_time.get()),3)  # time to raise/lower the flipper between open and close positions
    
    
def btn_srv_min_pulse(val):
    global b_min_pulse_width, b_servo_pos
    b_min_pulse_width = round(float(s_btn_srv_min_pulse.get()),3)  # bottom servo min pulse width
    disp.show_on_display('b_srv MIN PLS', str(b_min_pulse_width), fs1=15, y2=75, fs2=18)  # feedback is printed to the display
    pos = last_btn_srv_pos()
    servo.b_servo_create(b_min_pulse_width, b_max_pulse_width, pos)


def btn_srv_max_pulse(val):
    global b_max_pulse_width, b_servo_pos
    b_max_pulse_width = round(float(s_btn_srv_max_pulse.get()),3)  # bottom servo max pulse width
    disp.show_on_display('b_srv MAX PLS', str(b_max_pulse_width), fs1=15, y2=75, fs2=18)  # feedback is printed to the display
    pos = last_btn_srv_pos()
    servo.b_servo_create(b_min_pulse_width, b_max_pulse_width, pos)


def servo_CCW(val):
    global b_servo_CCW, b_servo_pos
    if t_servo_pos in ('close', 'open'):
        b_servo_CCW = round(float(s_btn_srv_CCW.get()),3)    # bottom servo position when fully CCW
        disp.show_on_display('b_srv CCW', str(b_servo_CCW), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
        servo.servo_to_pos('bottom', b_servo_CCW) # bottom servo is positioned to the slider value
        b_servo_pos = 'CCW'                       # string variable to track the last holder position
    else:
        disp.show_on_display('t_srv', 'BLOCKING', fs1=19, y2=75, fs2=18)  # feedback is printed to the display
    

def servo_home(val):
    global b_home, b_servo_pos
    if t_servo_pos in ('close', 'open'):
        b_home = round(float(s_btn_srv_home.get()),3)        # bottom servo home position
        disp.show_on_display('b_srv HOME', str(b_home), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
        servo.servo_to_pos('bottom', b_home)      # bottom servo is positioned to the slider value
        b_servo_pos = 'home'                      # string variable to track the last holder position
    else:
        disp.show_on_display('t_srv', 'BLOCKING', fs1=19, y2=75, fs2=18)  # feedback is printed to the display


def servo_CW(val):
    global b_servo_CW, b_servo_pos
    if t_servo_pos in ('close', 'open'):
        b_servo_CW = round(float(s_btn_srv_CW.get()),3)      # bottom servo position when fully CW
        disp.show_on_display('b_srv CW', str(b_servo_CW), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
        servo.servo_to_pos('bottom', b_servo_CW)  # bottom servo is positioned to the slider value
        b_servo_pos = 'CW'                        # string variable to track the last holder position
    else:
        disp.show_on_display('t_srv', 'BLOCKING', fs1=19, y2=75, fs2=18)  # feedback is printed to the display


def servo_rel_CCW(val):
    global b_rel_CCW
    b_rel_CCW = round(float(s_btn_srv_rel_CCW.get()),3)      # bottom servo position small rotation back from CCW, to release tension
    target = round(b_servo_CCW + b_rel_CCW,3)     # target position is defined by the b_servo_CCW AND b_rel_CCW
    servo.servo_to_pos('bottom', target)          # bottom servo is positioned to the slider value
    disp.show_on_display('b_rel_CCW', str(b_rel_CCW), fs1=19, y2=75, fs2=18)  # feedback is printed to the display


def servo_rel_CW(val):
    global b_rel_CW
    b_rel_CW = round(float(s_btn_srv_rel_CW.get()),3)        # bottom servo position small rotation back from CW, to release tension
    disp.show_on_display('b_rel_CW', str(b_rel_CW), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
    target = round(b_servo_CW - b_rel_CW,3)       # target position is defined by the b_servo_CW AND b_rel_CW
    servo.servo_to_pos('bottom', target)          # bottom servo is positioned to the slider value
    b_servo_pos = 'CW'                            # string variable to track the last holder position


def servo_extra_home_CCW(val):
    global b_extra_home_CCW, b_servo_pos
    b_extra_home_CCW = round(float(s_extra_home_CCW.get()),3)  # bottom servo position extra rotation at home, to release tension
    disp.show_on_display('b_extra CCW', str(b_extra_home_CCW), fs1=18, y2=75, fs2=18)  # feedback is printed to the display
    target = round(b_home + b_extra_home_CCW,3)   # target position is defined by the b_home AND b_extra_home_ccw
    servo.servo_to_pos('bottom', target)          # bottom servo is positioned to the slider value
    b_servo_pos = 'CW'                            # string variable to track the last holder position


def servo_extra_home_CW(val):
    global b_extra_home_CW, b_servo_pos
    b_extra_home_CW = round(float(s_extra_home_CW.get()),3)  # bottom servo position extra rotation at home, to release tension
    disp.show_on_display('b_extra CW', str(b_extra_home_CW), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
    target = round(b_home - b_extra_home_CW,3)    # target position is defined by the b_home AND b_extra_home_cw
    servo.servo_to_pos('bottom', target)          # bottom servo is positioned to the slider value
    b_servo_pos = 'CW'                            # string variable to track the last holder position


def servo_rotate_time(val):
    global b_rotate_time
    b_rotate_time = round(float(s_btn_srv_rotate_time.get()),3)  # time needed to the bottom servo to rotate about 90deg


def servo_rel_time(val):
    global b_rel_time
    b_rel_time = round(float(s_btn_srv_rel_time.get()),3)    # time to rotate slightly back, to release tensions


def servo_spin_time(val):
    global b_spin_time
    b_spin_time = round(float(s_btn_srv_spin_time.get()),3)  # time needed to the bottom servo to spin about 90deg


def crop_u(val):
    global y_u
    y_u = int(s_crop_u.get())                  # pixels to crop on upper image side
    disp.show_on_display('CROP UPPER', str(y_u), fs1=17, y2=75, fs2=18)  # feedback is printed to the display
    take_image(widgets_freeze=False)           # camera reads one frame


def crop_l(val):
    global x_l
    x_l = int(s_crop_l.get())                  # pixels to crop on left image side
    disp.show_on_display('CROP LEFT', str(x_l), fs1=18, y2=75, fs2=18)  # feedback is printed to the display
    take_image(widgets_freeze=False)           # camera reads one frame


def crop_r(val):
    global x_r
    x_r = int(s_crop_r.get())                  # pixels to crop on right image side
    disp.show_on_display('CROP RIGHT', str(x_r), fs1=17, y2=75, fs2=18)  # feedback is printed to the display
    take_image(widgets_freeze=False)           # camera reads one frame


def crop_b(val):
    global y_b
    y_b = int(s_crop_b.get())                  # pixels to crop on bottom image side
    disp.show_on_display('CROP BOTTOM', str(y_b), fs1=15, y2=75, fs2=18)  # feedback is printed to the display
    take_image(widgets_freeze=False)           # camera reads one frame


def warp_f_get(val):
    global warp_fraction
    warp_fraction = float(round(s_warp_f.get(),2))  # warping fraction
    disp.show_on_display('WARP FRACTION', str(warp_fraction), fs1=13, y2=75, fs2=18)  # feedback is printed to the display
    take_image(widgets_freeze=False)           # camera reads one frame


def warp_s_get(val):
    global warp_slicing
    warp_slicing = int(s_warp_s.get())         # Warping slicing
    disp.show_on_display('WARP SLICING', str(warp_slicing), fs1=15, y2=75, fs2=18)  # feedback is printed to the display
    take_image(widgets_freeze=False)           # camera reads one frame


def led_pwm_get(val):
    global cam_led_bright
    cam_led_bright = float(s_led_pwm.get())      # cam_led_bright 
    disp.show_on_display('LED PWM', str(round(cam_led_bright,2)), fs1=15, y2=75, fs2=18)  # feedback is printed to the display
    take_image(widgets_freeze=False)           # camera reads one frame
    
    
def expo_shift_get(val):
    global expo_shift
    expo_shift = float(s_expo_shift.get())      # cam_led_bright 
    disp.show_on_display('EXPO SHIFT', str(round(expo_shift,2)), fs1=15, y2=75, fs2=18)  # feedback is printed to the display
    reset_camera()
    take_image(widgets_freeze=False)           # camera reads one frame
######################################################################################################################





# ################################### functions to test the settings #################################################
def servo_init():
    """servo initialization."""
    global t_servo_pos, b_servo_pos
    
    servo_init=servo.servo_tune_via_gui(debug=False, start_pos = 'open')  # servos are initialized, and set to their starting positions
    if servo_init:                                # case the servo init has successed
        t_servo_pos = 'start'                     # string variable to track the last top_cover position
        b_servo_pos = 'home'                      # string variable to track the last holder position


def close_top_cover():
    global t_servo_pos
    servo.stop_release()                          # servos are made activate in case aren't
    if b_servo_pos in ('CCW', 'home', 'CW'):      # case the holder allows the Lifter to be moved freely
        disp.show_on_display('t_srv CLOSE', str(t_servo_close), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
        
        # send the close_cover settings request to the robot
        t_servo_pos = servo.close_cover(t_servo_close, t_servo_rel_delta, t_open_close_time, t_rel_time, test=True)
        disp.show_on_display('t_srv CLOSE', f'{t_servo_close} ({t_servo_rel_delta})', fs1=19, y2=75, fs2=18)  # feedback is printed to the display
        btm_srv_widgets_status()                  # updates the bottom servo related widgests status, based on top servo pos


def open_top_cover():
    global t_servo_pos
    servo.stop_release()                          # servos are made activate in case aren't
    if b_servo_pos in ('CCW', 'home', 'CW') and t_servo_pos != 'open':  # case the holder allows the Lifter to be moved freely
        t_servo_pos = servo.open_cover(t_servo_open, test=True) # send the open_cover settings request to the robot
        disp.show_on_display('t_srv OPEN', str(t_servo_open), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
        btm_srv_widgets_status()                  # updates the bottom servo related widgests status, based on top servo pos
        

def read_position():
    global t_servo_read, t_servo_pos
    servo.stop_release()                          # servos are made activate in case aren't
    if b_servo_pos in ('CCW', 'home', 'CW') and t_servo_pos != 'read':  # case the holder allows the Lifter to be moved freely
        t_servo_pos = servo.servo_to_pos('top', t_servo_read)  # send the request of top_cover to read position 
        disp.show_on_display('t_srv READ', str(t_servo_read), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
        btm_srv_widgets_status()                  # updates the bottom servo related widgests status, based on top servo pos


def flip_cube():
    global t_servo_pos
    servo.stop_release()                          # servos are made activate in case aren't
    if b_servo_pos in ('CCW', 'home', 'CW'):      # case the holder allows the Lifter to be moved freely
        t_servo_pos = servo.flip_toggle(t_servo_pos, t_servo_read, t_servo_flip)      # send the flip_test request to the robot
        if t_servo_pos == 'flip':                 # case the top_cover is in flip position
            disp.show_on_display('t_srv FLIP', str(t_servo_flip), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
        if t_servo_pos == 'read':                 # case the top_cover is in read position
            disp.show_on_display('t_srv READ', str(t_servo_read), fs1=19, y2=75, fs2=18)  # feedback is printed to the display
        btm_srv_widgets_status()                  # updates the bottom servo related widgests status, based on top servo pos


def ccw():
    global b_servo_pos
    servo.stop_release()                          # servos are made activate in case aren't
    
    if t_servo_pos == 'open':
        timer1 = b_spin_time
        if b_servo_pos == 'CW':
            timer1+=timer1
        if t_servo_pos in ('close', 'open') and b_servo_pos != 'CCW': # case the top_cover allows the holder to move freely
            target = round(b_servo_CCW + b_rel_CCW,3)
            b_servo_pos = servo.spin_out('CCW', target, 0, timer1, test=True ) # send the spin/rotate to CCW request to the robot
            disp.show_on_display('b_srv CCW', f'({target})' , fs1=19, y2=75, fs2=18)  # feedback is printed to the display
    elif t_servo_pos == 'close':
        timer1 = b_rotate_time
        if b_servo_pos == 'CW':
            timer1+=timer1
        if t_servo_pos in ('close', 'open') and b_servo_pos != 'CCW': # case the top_cover allows the holder to move freely
            b_servo_pos = servo.spin_out('CCW', b_servo_CCW, b_rel_CCW, timer1, test=True ) # send the spin/rotate to CCW request to the robot
            disp.show_on_display('b_srv CCW', f'{b_servo_CCW} ({round(b_servo_CCW+b_rel_CCW,3)})', fs1=19, y2=75, fs2=18)  # feedback is printed to the display

    
def home():
    global b_servo_pos
    
    if t_servo_pos in ('close', 'open') and b_servo_pos != 'home': # case the top_cover allows the holder to move freely
        servo.stop_release()                                 # servos are made activate in case aren't
        if t_servo_pos == 'open':
            timer1 = b_spin_time
            if b_servo_pos == 'CCW':
                b_servo_pos = servo.rotate_home('CW', b_home, 0, timer1, test=True)    # send the spin/rotate to HOME request to the robot
                disp.show_on_display('t_srv HOME', str(b_home), fs1=19, y2=75, fs2=18) # feedback is printed to the display
            elif b_servo_pos == 'CW':
                b_servo_pos = servo.rotate_home('CCW', b_home, 0, timer1, test=True)   # send the spin/rotate to HOME request to the robot
                disp.show_on_display('t_srv HOME', str(b_home), fs1=19, y2=75, fs2=18) # feedback is printed to the display
    
        elif t_servo_pos == 'close':
            timer1 = b_rotate_time
            if b_servo_pos == 'CCW':
                b_servo_pos = servo.rotate_home('CW', b_home, b_extra_home_CCW, timer1, test=True)  # send the spin/rotate to HOME request to the robot
                disp.show_on_display('t_srv HOME', f'{round(b_home+b_extra_home_CCW,3)} ({b_home})', fs1=19, y2=75, fs2=18)  # feedback is printed to the display
            elif b_servo_pos == 'CW':
                b_servo_pos = servo.rotate_home('CCW', b_home, b_extra_home_CW, timer1, test=True)  # send the spin/rotate to HOME request to the robot
                disp.show_on_display('t_srv HOME', f'{round(b_home-b_extra_home_CW,3)} ({b_home})', fs1=19, y2=75, fs2=18)  # feedback is printed to the display


def cw():
    global b_servo_pos
    
    if t_servo_pos in ('close', 'open') and b_servo_pos != 'CW':   # case the top_cover allows the holder to move freely
        servo.stop_release()                                 # servos are made activate in case aren't
        if t_servo_pos == 'open':
            timer1 = b_spin_time
            if b_servo_pos == 'CCW':
                timer1+=timer1
            target = round(b_servo_CW - b_rel_CW, 3)
            b_servo_pos = servo.spin_out('CW', target, 0, timer1, test=True) # send the spin/rotate to CW request to the robot
            disp.show_on_display('b_srv CCW', f'({target})', fs1=19, y2=75, fs2=18)  # feedback is printed to the display    
        elif t_servo_pos == 'close':
            timer1 = b_rotate_time
            if b_servo_pos == 'CCW':
                timer1+=timer1
            b_servo_pos = servo.spin_out('CW', b_servo_CW, b_rel_CW, timer1, test=True) # send the spin/rotate to CW request to the robot
            disp.show_on_display('b_srv CCW', f'{b_servo_CW} ({round(b_servo_CW-b_rel_CW,3)})', fs1=19, y2=75, fs2=18)  # feedback is printed to the display


def test():
    """call a fix sequence of robot movement that mimic a complete solving cycle.
        """
    
    for label in (top_srv_label, btn_srv_label, test_label, large_test_label, files_label): # iterations throught the labels in tuple
        disable_widgets(label, relief="sunken")  # widgets are disabled
    root.update()                                # forced a graphic update, upfront a long lasting function
    
    servo.stop_release()                         # servos are made activate in case aren't
    read_position()                              # top_servo positioned to read_position, as meant to be the start position after scanning
    result = servo.test_set_of_movements()       # function to verify the servos while solving a predefined robot movement string

    if result == 'stopped':                      # case the test has been interrupted
        servo_init()                             # servos are initialized
    elif result == 'completed':                  # case the test has reached the end
        open_top_cover()                         # top servo is positioned to open
        time.sleep(0.5)                          # little delay to give the servo time to reach the target
        home()                                   # bottom servo is positioned to home
        
    for label in (top_srv_label, btn_srv_label): # iterations throught the labels (of sliders) in tuple
        enable_widgets(label, relief="raised")   # widgets are enabled
    
    for label in (test_label, large_test_label, files_label):  # iterations throught the labels (of buttons) in tuple
        enable_widgets(label, relief="raised")   # widgets are enabled
    
    btm_srv_widgets_status()                     # bottom servo widgets are updated according to the top servo position



def btm_srv_widgets_status():
    """function to enable/disable the bottom servo related widgets according to the top_servo (lifter) position."""
    global t_servo_pos
    
    if t_servo_pos == 'close' or t_servo_pos == 'open':  # case the lifter prevents the holder rotation 
        ccw_btn["relief"] = "raised"            # button is set back raised
        ccw_btn["state"] = "normal"             # button is set back normal
        home_btn["relief"] = "raised"           # button is set back raised
        home_btn["state"] = "normal"            # button is set back normal
        cw_btn["relief"] = "raised"             # button is set back raised
        cw_btn["state"] = "normal"              # button is set back normal
        enable_widgets(btn_srv_label, relief="raised")
    
    else:                                       # case the lifter is not blocking the holder rotation
        ccw_btn["relief"] = "sunken"            # button is set sunken
        ccw_btn["state"] = "disabled"           # button is set disabled
        home_btn["relief"] = "sunken"           # button is set sunken
        home_btn["state"] = "disabled"          # button is set disabled
        cw_btn["relief"] = "sunken"             # button is set sunken
        cw_btn["state"] = "disabled"            # button is set disabled
        disable_widgets(btn_srv_label, relief="sunken")
######################################################################################################################





# #################### functions to manage the GUI windows ###########################################################
def goto_camWindow():
    """function to bring the cam settings Window on top."""
    show_window(camWindow)


def goto_servoWindow():
    """function to bring the servo settings Window on top."""
    show_window(servoWindow)


def disable_widgets(parent, relief):
    for child in parent.winfo_children():      # iteration through children widgets in parent
            child["relief"] = relief           # slider is set sunken
            child["state"] = "disabled"        # slider is set disabled


def enable_widgets(parent, relief):
    for child in parent.winfo_children():      # iteration through children widgets in parent
            child["relief"] = relief           # slider is set flat
            child["state"] = "normal"          # slider is set normal


def show_window(window, startup=False):
    """Function to bring to the front the window in argument."""
    global servoWindow_ontop
    if window==servoWindow:                    # case the request is to show the servo settings windows
        window.tkraise()                       # servo settings windows is raised on top
        servoWindow_ontop=True                 # boolean of servo settings windows being on top is set true
        root.geometry('+0+40')                 # windows is presented at top-left of the screen
        if not startup:                        # case the servoWindow is called after the camWindow
            root.resizable(1,1)                # root windows is set as resizable
            root.geometry(f'{srv_w_w}x{srv_w_h}')   # servo windows size set to the dimensions initially calculated
            cv2.destroyAllWindows()            # all open windows are closed
        disp.show_on_display('SERVOS', 'TUNING', fs1=27, y2=75, fs2=27)  # feedback is printed to the display
        disp.set_backlight(1)                  # display backlight is turned on, in case it wasn't
        return                                 # function in closed

    elif window==camWindow:                    # case the request is to show the cam setting windows
        read_position()                        # top_cover positioned to read position
        window.tkraise()                       # picamera setting windows is raised on top
        servoWindow_ontop=False                # boolean of servo settings windows being on top is set false
        w = 1100                               # window width
        h = srv_w_h                            # windows height
        root.resizable(1,1)                    # root windows is set to be resizable on x and y
        root.geometry(f'{w}x{h}+{int(ws-w)}+40') # windows is initially presented at top-right of the screen
        root.update()                          # forced a windows updated prior a long process
        update_cam_sliders()                   # updates the sliders to the variable values from settings
        root.update()                          # forced a windows updated prior a long process
        disp.show_on_display('CAMERA', 'TUNING', fs1=26, y2=75, fs2=27)  # feedback is printed to the display
        disp.set_backlight(1)                  # display backlight is turned on, in case it wasn't
        take_image(refresh=10)                 # camera takes one frame and displayes it
######################################################################################################################







# ####################################################################################################################
# ############################### Starting variables  ################################################################
# ####################################################################################################################

printout = False                             # boolean to print the settings to the terminal at script starting
servoWindow_ontop=True                       # boolean to track which window is on top
picamera_initialized = False                 # boolean to track if the camera has been initialized
startup = True                               # boolean to track the first time window generation

folder = pathlib.Path().resolve()            # active folder (should be home/pi/cubotino)
eth_mac = get_mac_address().lower()          # board mac address is retrieved
macs_AF = get_macs_AF()                      # mac addresses of AF bots are retrieved

if eth_mac in macs_AF:                       # case board mac is in macs_AF tupple
    mac_pos = macs_AF.index(eth_mac)         # mac addreess position in the tupple
#     print("Board's mac is listed in macs_AF.txt file")      # feedback is printed to the terminal
else:                                        # case board mac is not in macs_AF tupple
    mac_pos = ''                             # the mac addreess position is set empty
#     print("Board's mac is not listed in macs_AF.txt file")  # feedback is printed to the terminal
    
srv_settings  = read_servo_settings_file()   # servos settings are retrieved from the json settings files
upload_servo_settings(srv_settings)          # servos settings are loaded to global
servo_init()                                 # servos are initialized

cam_settings  = read_cam_settings_file()     # cam settings are retrieved from the json settings files
upload_cam_settings(cam_settings)            # cam settings are loaded to global






# ####################################################################################################################
# ############################### GUI High level part ################################################################
# ####################################################################################################################
root = tk.Tk()                            # initialize tkinter as root 
root.title("CUBOTino servos setting")     # name is assigned to GUI root
root.geometry('+0+40')                    # windows is initially presented at top-left of the screen


root.rowconfigure(0, weight=1)            # root is set to have 1 row of  weight=1
root.columnconfigure(0,weight=1)          # root is set to have 1 column of weight=1
root.resizable(1,1)

# two windows
servoWindow=tk.Frame(root)                # a first windows (called servoWindow) is derived from root
camWindow=tk.Frame(root)                  # a second windows (called camWindow) is derived from root
for window in (servoWindow, camWindow):   # iteration over the two defined windows
    window.grid(row=0,column=0,sticky='nsew')  # each window goes to the only row/column, and centered

show_window(servoWindow, startup)         # the first window (mainWindow) is the one that will initially appear
######################################################################################################################





# ####################################################################################################################
# ############################### GUI Low level part #################################################################
# ####################################################################################################################


#### top servo related widgets ####
top_srv_label = tk.LabelFrame(servoWindow, text="Top cover - servo settings", labelanchor="nw", font=("Arial", "14"))
top_srv_label.grid(row=0, column=0, rowspan=1, columnspan=4, sticky="n", padx=20, pady=30)


s_top_srv_close = tk.Scale(top_srv_label, label="CLOSE", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=1, to_=-1, resolution=0.02)#, command=servo_close)                           
s_top_srv_close.grid(row=0, column=0, sticky="w", padx=12, pady=5)
s_top_srv_close.set(t_servo_close)
s_top_srv_close.bind("<ButtonRelease-1>", servo_close)


s_top_srv_rel_delta = tk.Scale(top_srv_label, label="RELEASE (at close)", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0, to_=0.2, resolution=0.02)                          
s_top_srv_rel_delta.grid(row=0, column=1, sticky="w", padx=12, pady=5)
s_top_srv_rel_delta.set(t_servo_rel_delta)
s_top_srv_rel_delta.bind("<ButtonRelease-1>", servo_release)


s_top_srv_open = tk.Scale(top_srv_label, label="OPEN", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=1, to_=-1, resolution=0.02)
s_top_srv_open.grid(row=0, column=2, sticky="w", padx=12, pady=5)
s_top_srv_open.set(t_servo_open)
s_top_srv_open.bind("<ButtonRelease-1>", servo_open)


s_top_srv_read = tk.Scale(top_srv_label, label="READ", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=1, to_=-1, resolution=0.02)
s_top_srv_read.grid(row=0, column=3, sticky="w", padx=12, pady=5)
s_top_srv_read.set(t_servo_read)
s_top_srv_read.bind("<ButtonRelease-1>", servo_read)


s_top_srv_flip = tk.Scale(top_srv_label, label="FLIP", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=1, to_=-1, resolution=0.02)
s_top_srv_flip.grid(row=0, column=4, sticky="w", padx=12, pady=5)
s_top_srv_flip.set(t_servo_flip)
s_top_srv_flip.bind("<ButtonRelease-1>", servo_flip)


s_top_srv_rel_time = tk.Scale(top_srv_label, label="RELEASE TIME at close (s)", font=('arial','14'),
                                     orient='horizontal', relief='raised', length=320, from_=0, to_=0.5,
                                     resolution=0.02, command=servo_rel_close)
s_top_srv_rel_time.grid(row=1, column=0, sticky="w", padx=12, pady=15)
s_top_srv_rel_time.set(t_rel_time)


s_top_srv_flip_to_close_time = tk.Scale(top_srv_label, label="TIME: flip > close (s)", font=('arial','14'),
                                        orient='horizontal', relief='raised', length=320, from_=0, to_=1, 
                                        resolution=0.02, command=flip_to_close_time)
s_top_srv_flip_to_close_time.grid(row=1, column=1, sticky="w", padx=12, pady=15)
s_top_srv_flip_to_close_time.set(t_flip_to_close_time)


s_top_srv_close_to_flip_time = tk.Scale(top_srv_label, label="TIME: close > flip (s)", font=('arial','14'),
                                        orient='horizontal', relief='raised', length=320, from_=0, to_=1,
                                        resolution=0.02, command=close_to_flip_time)
s_top_srv_close_to_flip_time.grid(row=1, column=2, sticky="w", padx=12, pady=15)
s_top_srv_close_to_flip_time.set(t_close_to_flip_time)


s_top_srv_flip_open_time = tk.Scale(top_srv_label, label="TIME: flip <> open (s)", font=('arial','14'),
                                    orient='horizontal', relief='raised', length=320, from_=0, to_=1,
                                    resolution=0.02, command=flip_open_time)
s_top_srv_flip_open_time.grid(row=1, column=3, sticky="w", padx=12, pady=15)
s_top_srv_flip_open_time.set(t_flip_open_time)


s_top_srv_open_close_time = tk.Scale(top_srv_label, label="TIME: open <> close(s)", font=('arial','14'),
                                     orient='horizontal', relief='raised', length=320, from_=0, to_=1,
                                     resolution=0.02, command=open_close_time)
s_top_srv_open_close_time.grid(row=1, column=4, sticky="w", padx=12, pady=15)
s_top_srv_open_close_time.set(t_open_close_time)








#### bottom servo related widgets ####
btn_srv_label = tk.LabelFrame(servoWindow, text="Cube holder - servo settings",
                                   labelanchor="nw", font=("Arial", "14"))
btn_srv_label.grid(row=1, column=0, rowspan=1, columnspan=4, sticky="n", padx=20, pady=30)


s_btn_srv_min_pulse = tk.Scale(btn_srv_label, label="MIN PULSE WIDTH", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0.2, to_=1.2, resolution=0.02)
s_btn_srv_min_pulse.grid(row=0, column=0, sticky="w", padx=12, pady=5)
s_btn_srv_min_pulse.set(b_min_pulse_width)
s_btn_srv_min_pulse.bind("<ButtonRelease-1>", btn_srv_min_pulse)



s_btn_srv_CCW = tk.Scale(btn_srv_label, label="CCW", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=-1, to_=-0.5, resolution=0.02)
s_btn_srv_CCW.grid(row=0, column=1, sticky="w", padx=12, pady=5)
s_btn_srv_CCW.set(b_servo_CCW)
s_btn_srv_CCW.bind("<ButtonRelease-1>", servo_CCW)


s_btn_srv_home = tk.Scale(btn_srv_label, label="HOME", font=('arial','14'), orient='horizontal',
                          width=30, relief='raised', length=320, from_=-0.5, to_=0.5, resolution=0.02)                           
s_btn_srv_home.grid(row=0, rowspan=2, column=2, sticky="w", padx=12, pady=5)
s_btn_srv_home.set(b_home)
s_btn_srv_home.bind("<ButtonRelease-1>", servo_home)


s_btn_srv_CW = tk.Scale(btn_srv_label, label="CW", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0.5, to_=1, resolution=0.02)
s_btn_srv_CW.grid(row=0, column=3, sticky="w", padx=12, pady=5)
s_btn_srv_CW.set(b_servo_CW)
s_btn_srv_CW.bind("<ButtonRelease-1>", servo_CW)


s_btn_srv_max_pulse = tk.Scale(btn_srv_label, label="MAX PULSE WIDTH", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=1.8, to_=2.8, resolution=0.02)
s_btn_srv_max_pulse.grid(row=0, column=4, sticky="w", padx=12, pady=5)
s_btn_srv_max_pulse.set(b_max_pulse_width)
s_btn_srv_max_pulse.bind("<ButtonRelease-1>", btn_srv_max_pulse)


s_btn_srv_rel_CCW = tk.Scale(btn_srv_label, label="RELEASE at CCW  -->", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0, to_=0.2, resolution=0.02)
s_btn_srv_rel_CCW.grid(row=1, column=0, sticky="w", padx=12, pady=5)
s_btn_srv_rel_CCW.set(b_rel_CCW)
s_btn_srv_rel_CCW.bind("<ButtonRelease-1>", servo_rel_CCW)


s_extra_home_CW = tk.Scale(btn_srv_label, label="EXTRA HOME from CW  <--", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0.2, to_=0, resolution=0.02)
s_extra_home_CW.grid(row=1, column=1, sticky="w", padx=12, pady=5)
s_extra_home_CW.set(b_extra_home_CW)
s_extra_home_CW.bind("<ButtonRelease-1>", servo_extra_home_CW)


s_extra_home_CCW = tk.Scale(btn_srv_label, label="EXTRA HOME from CCW  -->", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0, to_=0.2, resolution=0.02)                           
s_extra_home_CCW.grid(row=1, column=3, sticky="w", padx=12, pady=5)
s_extra_home_CCW.set(b_extra_home_CCW)
s_extra_home_CCW.bind("<ButtonRelease-1>", servo_extra_home_CCW)


s_btn_srv_rel_CW = tk.Scale(btn_srv_label, label="RELEASE at CW  <--", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0.2, to_=0, resolution=0.02)
s_btn_srv_rel_CW.grid(row=1, column=4, sticky="w", padx=12, pady=5)
s_btn_srv_rel_CW.set(b_rel_CW)
s_btn_srv_rel_CW.bind("<ButtonRelease-1>", servo_rel_CW)



s_btn_srv_spin_time = tk.Scale(btn_srv_label, label="TIME: spin (s)", font=('arial','14'),
                                        orient='horizontal', relief='raised', length=320, from_=0, to_=1,
                                        resolution=0.02, command=servo_spin_time)
s_btn_srv_spin_time.grid(row=2, column=1, sticky="w", padx=12, pady=15)
s_btn_srv_spin_time.set(b_spin_time)


s_btn_srv_rotate_time = tk.Scale(btn_srv_label, label="TIME: rotate (s)", font=('arial','14'),
                                    orient='horizontal', relief='raised', length=320, from_=0, to_=1,
                                    resolution=0.02, command=servo_rotate_time)
s_btn_srv_rotate_time.grid(row=2, column=2, sticky="w", padx=12, pady=15)
s_btn_srv_rotate_time.set(b_rotate_time)


s_btn_srv_rel_time = tk.Scale(btn_srv_label, label="TIME: release (s)", font=('arial','14'),
                                     orient='horizontal', relief='raised', length=320, from_=0, to_=1,
                                     resolution=0.02, command=servo_rel_time)
s_btn_srv_rel_time.grid(row=2, column=3, sticky="w", padx=12, pady=15)
s_btn_srv_rel_time.set(b_rel_time)








#### test settings related widgets ####
test_label = tk.LabelFrame(servoWindow, text="Test (sliders settings)", labelanchor="nw", font=("Arial", "14"))
test_label.grid(row=2, column=0, rowspan=1, columnspan=1, sticky="w", padx=20, pady=30)

close_btn = tk.Button(test_label, text="CLOSE", height=1, width=15, state="normal", command= close_top_cover)
close_btn.configure(font=("Arial", "12"))
close_btn.grid(row=0, column=0, sticky="n", padx=20, pady=10)

open_btn = tk.Button(test_label, text="OPEN", height=1, width=15, state="normal", command= open_top_cover)
open_btn.configure(font=("Arial", "12"))
open_btn.grid(row=0, column=1, sticky="n", padx=20, pady=10)

read_btn = tk.Button(test_label, text="READ", height=1, width=15, state="normal", command= read_position)
read_btn.configure(font=("Arial", "12"))
read_btn.grid(row=0, column=2, sticky="n", padx=20, pady=10)

flip_btn = tk.Button(test_label, text="FLIP  (toggle)", height=1, width=18, state="normal", command= flip_cube)
flip_btn.configure(font=("Arial", "12"))
flip_btn.grid(row=0, column=3, sticky="n", padx=20, pady=10)

ccw_btn = tk.Button(test_label, text="CCW", height=1, width=15, state="normal", command= ccw)
ccw_btn.configure(font=("Arial", "12"))
ccw_btn.grid(row=1, column=0, sticky="n", padx=20, pady=10)

home_btn = tk.Button(test_label, text="HOME", height=1, width=15, state="normal", command= home)
home_btn.configure(font=("Arial", "12"))
home_btn.grid(row=1, column=1, sticky="n", padx=20, pady=10)

cw_btn = tk.Button(test_label, text="CW", height=1, width=15, state="normal", command= cw)
cw_btn.configure(font=("Arial", "12"))
cw_btn.grid(row=1, column=2, sticky="n", padx=20, pady=10)

#######################################################################################




#### saving settings ####
files_label = tk.LabelFrame(servoWindow, text="Settings files", labelanchor="nw", font=("Arial", "14"))
files_label.grid(row=2, column=1, rowspan=1, columnspan=1, sticky="w", padx=20, pady=30)


reset_btn = tk.Button(files_label, text="LOAD PREVIOUS SETTING", height=1, width=30, state="normal", command= load_previous_servo_settings)
reset_btn.configure(font=("Arial", "12"))
reset_btn.grid(row=0, column=0, sticky="n", padx=20, pady=10)


save_btn = tk.Button(files_label, text="SAVE SETTINGS", height=1, width=30, state="normal", command= save_new_servo_settings)
save_btn.configure(font=("Arial", "12"))
save_btn.grid(row=1, column=0, sticky="n", padx=20, pady=10)




#### Large test ####
large_test_label = tk.LabelFrame(servoWindow, text="Full test", labelanchor="nw", font=("Arial", "14"))
large_test_label.grid(row=2, column=2, rowspan=1, columnspan=1, sticky="w", padx=20, pady=30)

test_btn = tk.Button(large_test_label, text="LONG TEST\n(on saved settings)", height=4, width=18, state="normal", command= test)
test_btn.configure(font=("Arial", "12"))
test_btn.grid(row=0, column=0, sticky="w", padx=20, pady=10)




#### change window ####
to_camera_btn = tk.Button(servoWindow, text="CAMERA\nSETTINGS\nwindow", height=4, width=16, state="normal", command= goto_camWindow)
to_camera_btn.configure(font=("Arial", "12"))
to_camera_btn.grid(row=2, column=3, sticky="w", padx=20, pady=10)






# ####################################################################################################################
# ###############################     camera window widgets     ######################################################
# ####################################################################################################################

#### bottom servo related widgets ####
crop_label = tk.LabelFrame(camWindow, text="Image cropping",
                                   labelanchor="nw", font=("Arial", "14"))
crop_label.grid(row=0, column=0, rowspan=1, columnspan=3, sticky="n", padx=20, pady=30)


s_crop_u = tk.Scale(crop_label, label="CROP UPPER SIDE", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0, to_=200, resolution=2)
s_crop_u.grid(row=0, column=1, sticky="w", padx=12, pady=5)
s_crop_u.set(y_u)
s_crop_u.bind("<ButtonRelease-1>", crop_u)


s_crop_l = tk.Scale(crop_label, label="CROP LEFT SIDE", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0, to_=200, resolution=2)
s_crop_l.grid(row=1, column=0, sticky="w", padx=12, pady=5)
s_crop_l.set(x_l)
s_crop_l.bind("<ButtonRelease-1>", crop_l)


s_crop_r = tk.Scale(crop_label, label="CROP RIGHT SIDE", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0, to_=200, resolution=2)
s_crop_r.grid(row=1, column=2, sticky="w", padx=12, pady=5)
s_crop_r.set(x_r)
s_crop_r.bind("<ButtonRelease-1>", crop_r)


s_crop_b = tk.Scale(crop_label, label="CROP BOTTOM SIDE", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0, to_=200, resolution=2)
s_crop_b.grid(row=2, column=1, sticky="w", padx=12, pady=5)
s_crop_b.set(y_b)
s_crop_b.bind("<ButtonRelease-1>", crop_b)


read_camera_btn = tk.Button(crop_label, text="REFRESH CAMERA", height=1, width=26, state="normal", command= take_image)
read_camera_btn.configure(font=("Arial", "14"))
read_camera_btn.grid(row=1, column=1, sticky="nsew", padx=20, pady=10)



warp_label = tk.LabelFrame(camWindow, text="Image warping",
                                   labelanchor="nw", font=("Arial", "14"))
warp_label.grid(row=1, column=0, rowspan=1, columnspan=3, sticky="n", padx=20, pady=30)


s_warp_f = tk.Scale(warp_label, label="WARP FRACTION", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=20, to_=3, resolution=0.1)
s_warp_f.grid(row=0, column=0, sticky="w", padx=12, pady=5)
s_warp_f.set(warp_fraction)
s_warp_f.bind("<ButtonRelease-1>", warp_f_get)


s_warp_s = tk.Scale(warp_label, label="WARP SLICING", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0.1, to_=30, resolution=0.1)
s_warp_s.grid(row=0, column=1, sticky="w", padx=12, pady=5)
s_warp_s.set(warp_slicing)
s_warp_s.bind("<ButtonRelease-1>", warp_s_get)


read_camera_btn2 = tk.Button(warp_label, text="REFRESH CAMERA", height=1, width=26, state="normal", command= take_image)
read_camera_btn2.configure(font=("Arial", "14"))
read_camera_btn2.grid(row=0, column=2, sticky="nsew", padx=20, pady=10)


# Led PWM
expo_label = tk.LabelFrame(camWindow, text="Led PWM and camera exposure shift",
                                   labelanchor="nw", font=("Arial", "14"))
expo_label.grid(row=2, column=0, rowspan=1, columnspan=3, sticky="n", padx=20, pady=30)

s_led_pwm = tk.Scale(expo_label, label="LED PWM SETTING", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=0, to_=0.3, resolution=0.01)
s_led_pwm.grid(row=0, column=0, sticky="w", padx=12, pady=5)
s_led_pwm.set(cam_led_bright)
s_led_pwm.bind("<ButtonRelease-1>", led_pwm_get)

# Expo shift
s_expo_shift = tk.Scale(expo_label, label="EXPOSURE SHIFT (only OS 11)", font=('arial','14'), orient='horizontal',
                          relief='raised', length=320, from_=-3, to_=1, resolution=0.1)
s_expo_shift.grid(row=0, column=1, sticky="w", padx=12, pady=5)
s_expo_shift.set(expo_shift)
s_expo_shift.bind("<ButtonRelease-1>", expo_shift_get)


read_camera_btn3 = tk.Button(expo_label, text="REFRESH CAMERA", height=1, width=26, state="normal", command= take_image)
read_camera_btn3.configure(font=("Arial", "14"))
read_camera_btn3.grid(row=0, column=2, sticky="nsew", padx=20, pady=10)


#### saving settings ####
cam_files_label = tk.LabelFrame(camWindow, text="Settings files", labelanchor="nw", font=("Arial", "14"))
cam_files_label.grid(row=3, column=0, rowspan=1, columnspan=1, sticky="w", padx=20, pady=30)


reset_btn = tk.Button(cam_files_label, text="LOAD PREVIOUS SETTING", height=1, width=30, state="normal", command= load_previous_cam_settings)
reset_btn.configure(font=("Arial", "12"))
reset_btn.grid(row=0, column=0, sticky="w", padx=20, pady=10)


save_btn = tk.Button(cam_files_label, text="SAVE SETTINGS", height=1, width=30, state="normal", command= save_new_cam_settings)
save_btn.configure(font=("Arial", "12"))
save_btn.grid(row=0, column=1, sticky="w", padx=20, pady=10)


#### change window ####
to_servo_btn = tk.Button(camWindow, text="SERVO\nSETTINGS\nwindow", height=4, width=16, state="normal", command= goto_servoWindow)
to_servo_btn.configure(font=("Arial", "12"))
to_servo_btn.grid(row=3, column=2, sticky="e", padx=20, pady=10)





# ####################################################################################################################
# ###############################     main part   ####################################################################
# ####################################################################################################################

btm_srv_widgets_status()                         # bottom servo widgets are updated according to the top servo position

root.update()                                    # forces a graphical update
srv_w_w = root.winfo_width()                     # retrieves the window width dimension set automatically 
srv_w_h = root.winfo_height()                    # retrieves the window height dimension set automatically 
ws = root.winfo_screenwidth()                    # retrieves the width of the screen
hs = root.winfo_screenheight()                   # retrieves the height of the screen

root.protocol("WM_DELETE_WINDOW", on_closing)    # closing the GUI
root.mainloop()                                  # tkinter main loop

