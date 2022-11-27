#!/usr/bin/python
# coding: utf-8

""" 
#############################################################################################################
#  Andrea Favero, 26 November 2022
#
#
#  This code relates to CUBOTino autonomous, a very small and simple Rubik's cube solver robot 3D printed.
#  CUBOTino autonomous is the 'Top version', of the CUBOTino robot series.
#  Demo at https://youtu.be/dEOLhvVMcUg
#  Instructions:https://www.instructables.com/CUBOTino-Autonomous-Small-3D-Printed-Rubiks-Cube-R/
#
#  This is the core script, that interracts with few other files.
#  Many functions of this code have been developed on 2021, for my previous robot (https://youtu.be/oYRXe4NyJqs).
#
#  The cube status is detected via a camera system (piCamera) and OpenCV .
#  Kociemba solver is used for the cube solution (from: https://github.com/hkociemba/RubiksCube-TwophaseSolver)
#  Credits to Mr. Kociemba for his great job !
#  Search for CUBOTino autonomous on www.intructable.com, to find more info about this robot.
#
#  Developped on:
#  - Raspberry pi Zero 2
#  - Raspberry Pi OS (Legacy) A port of Debian Buster with security updates and desktop environment
#    (Linux raspberry 5.10.103-v7+ #1529 SMP Tue 8 12:21:37 GMT 2022 armv71 GNU/Linux)
#  - OpenCV cv2 ver: 4.1.0
#  - PiCamera v1.3
#  - Numpy: 1.21.4
#
#############################################################################################################
"""






###################### pre-setting a number of scrambling and solving cycles   ##################
import argparse
parser = argparse.ArgumentParser(description='Scrambling and solving cycles')   # argument parser object creation

parser.add_argument("--cycles", type=int, 
                    help="Input the number of automated scrambling and solving cycles") # argument is added to the parser

parser.add_argument("--pause", type=int, 
                    help="Input the pause time, in secs, between automated cycles") # argument is added to the parser

args = parser.parse_args()              # argument parsed assignement
# ###############################################################################################






def import_parameters():
    """ Function to import parameters from a json file, to make easier to list/document/change the variables
        that are expected to vary on each robot."""
       
    global frameless_cube, camera_width_res, camera_hight_res
    global kl, x_l, x_r, y_u, y_b, warp_fraction, warp_slicing, square_ratio, rhombus_ratio
    global delta_area_limit, sv_max_moves, sv_max_time, collage_w, marg_coef, cam_led_bright
    global detect_timeout, show_time, warn_time, quit_time, cover_self_close
    global pathlib
    
    # convenient choice for Andrea Favero, to upload the settings fitting my robot, via mac address check
    import os.path, pathlib, json                                 # libraries needed for the json, and parameter import
    from getmac import get_mac_address                            # library to get the device MAC ddress
        
    folder = pathlib.Path().resolve()                             # active folder (should be home/pi/cube)  
    eth_mac = get_mac_address()                                   # mac address is retrieved
    if eth_mac == 'e4:5f:01:8d:59:97':                            # case the script is running on AF (Andrea Favero) robot
        fname = os.path.join(folder,'Cubotino_T_settings_AF.txt') # AF robot settings (do not use these at the start)
    elif eth_mac == 'e4:5f:01:8b:5f:e5':                          # case the script is running on AF (Andrea Favero) robot
        fname = os.path.join(folder,'Cubotino_T_settings_AF2.txt')# AF second robot settings (do not use these at the start)
    else:                                                         # case the script is not running on AF (Andrea Favero) robot
        fname = os.path.join(folder,'Cubotino_T_settings.txt')    # folder and file name for the settings, to be tuned
    
    if os.path.exists(fname):                                     # case the settings file exists
        with open(fname, "r") as f:                               # settings file is opened in reading mode
            settings = json.load(f)                               # json file is parsed to a local dict variable
            # NOTE: in case of git pull, the settings file will be overwritten, the backup file not
        
        if debug:                                                 # case debug variable is set true on __main_
            print('\nimporting settings from the text file:', fname)    # feedback is printed to the termina
            print('\nimported parameters: ')                      # feedback is printed to the terminal
            for parameter, setting in settings.items():           # iteration over the settings dict
                print(parameter,': ', setting)                    # feedback is printed to the terminal
            print()
            
        os.umask(0) # The default umask is 0o22 which turns off write permission of group and others
        backup_fname = os.path.join(folder,'Cubotino_T_settings_backup.txt')          # folder and file name for the settings backup
        with open(os.open(backup_fname, os.O_CREAT | os.O_WRONLY, 0o777), 'w') as f:  # settings_backup file is opened in writing mode
            if debug:                                             # case debug variable is set true on __main_
                print('copy of settings parameter is saved as backup at: ', backup_fname)    # feedback is printed to the terminal
            f.write(json.dumps(settings, indent=0))               # content of the setting file is saved in another file, as backup
            # NOTE: in case of git pull, the settings file will be overwritten, the backup file not
        
        if debug:                                                 # case debug variable is set true on __main_
            print('\nimported parameters are saved to the backup file\n') # feedback is printed to the terminal


        try:                                  # tentative
            if 'frameless_cube' in settings:  # case the frameless parameter key is available at settings file (back compatibility...)
                if settings['frameless_cube'].lower().strip() == 'false':  # case frameless_cube parameter is a string == false
                    frameless_cube = 'false'                               # cube with black frame around the facelets
                elif settings['frameless_cube'].lower().strip() == 'true': # case frameless_cube parameter is a string == true
                    frameless_cube = 'true'                                # cube without black frame around the facelets
                elif settings['frameless_cube'].lower().strip() == 'auto': # case frameless_cube parameter is a string == auto
                    frameless_cube = 'auto'                                # cube without black frame around the facelets
                else:                                                      # case the frameless parameter is not 'false', 'true' or 'auto'
                    print('\n\nAttention: Wrong frameless_cube parameter: It should be "true", "false" or "auto".\n')  # feedback is printed to the terminal
                    return False, ''   # parameter import process is interrupted,  return robot_init_status variable False and empty parameters
            else:                      # case the frameless parameter key is not available at settings file (back compatibility...)
                print("NOTE: Add 'frameless_cube' parameter in yout Cubotino_T_settings.txt file.") # feedback is printed to the terminal
                print("NOTE: When absent, the classic cube is considered.\n\n") # feedback is printed to the terminal
                frameless_cube = 'false'                                       # cube with black frame around the facelets
            
            camera_width_res = int(settings['camera_width_res'])      # Picamera resolution on width 
            camera_hight_res = int(settings['camera_hight_res'])      # Picamera resolution on heigh
            kl = float(settings['kl'])                                # coff. for PiCamera stabili acceptance
            x_l = int(settings['x_l'])                                # image crop on left (before warping)
            x_r = int(settings['x_r'])                                # image crop on right (before warping)
            y_u = int(settings['y_u'])                                # image crop on top (before warping)
            y_b = int(settings['y_b'])                                # image crop on bottom (before warping)
            warp_fraction = float(settings['warp_fraction'])          # coeff for warping the image
            warp_slicing = float(settings['warp_slicing'])            # coeff for cropping the bottom warped image
            if warp_slicing == 0:                                     # case the parameter equals to zero
                warp_slicing = 0.1                                    # the parameter is set to 0.1
            square_ratio = float(settings['square_ratio'])            # acceptance threshold for square sides difference
            rhombus_ratio = float(settings['rhombus_ratio'])          # acceptance threshold for rhombus axes difference
            delta_area_limit = float(settings['delta_area_limit'])    # acceptance threshold for facelet area dev from median
            sv_max_moves = int(settings['sv_max_moves'])              # max moves requested to the Kociemba solver
            sv_max_time = float(settings['sv_max_time'])              # timeout requested to the Kociemba solver
            collage_w = int(settings['collage_w'])                    # image width for unfolded cube collage
            marg_coef = float(settings['marg_coef'])                  # cropping margin arounf faces for immage collage
            cam_led_bright = float(settings['cam_led_bright'])        # PWM level for the 3W led at Top_cover
            detect_timeout = int(settings['detect_timeout'])          # timeout for cube status detection
            show_time = int(settings['show_time'])                    # showing time of the unfolded cube image collage
            warn_time = float(settings['warn_time'])                  # touch button pressing time before get a worning
            quit_time = float(settings['quit_time'])                  # touch button pressing time before the quit process
            
            if 'cover_self_close' in settings:      # case the cover_self_close key is available at settings file (back compatibility...)
                if settings['cover_self_close'].lower().strip() == 'false':  # case cover_self_close parameter is a string == false
                    cover_self_close = False                                 # cover_self_close parameter is set boolean False
                elif settings['cover_self_close'].lower().strip() == 'true': # case cover_self_close parameter is a string == true
                    cover_self_close = True                                  # cover_self_close parameter is set boolean True
                else:                                                        # case the frameless parameter is not 'false', 'true' or 'auto'
                    print('\n\nAttention: Wrong cover_self_close parameter: It should be "true" or "false."\n')  # feedback is printed to the terminal
                    return False, ''   # parameter import process is interrupted,  return robot_init_status variable False and empty parameters
            else:                                   # case the cover_self_close key is not available at settings file (back compatibility...)
                print("NOTE: Add 'cover_self_close' parameter in yout Cubotino_T_settings.txt file.\n") # feedback is printed to the terminal
                cover_self_close = False            # cover_self_close parameter is set boolean False
            
            return True, settings
            
        
        except:   # exception will be raised if json keys differs, or parameters cannot be converted to int/float
            print('error on converting the imported parameters to int, float or string')   # feedback is printed to the terminal                                  
            return False, ''                                    # return robot_init_status variable, that is False
    
    else:                                                       # case the settings file does not exists, or name differs
        print('could not find Cubotino_T_servo_settings.txt')   # feedback is printed to the terminal                                  
        return False, ''                                        # return robot_init_status variable, that is False


def import_libraries():
    """ Import of the needed libraries.
        These librries are imported after those needed for the display management.
        Kociemba solver is tentatively imported considering three installation/copy methods."""
    
    global camera_set_gains, dist, PiRGBArray, PiCamera, servo, rm, GPIO, median, dt, sv, cubie, np, math, time, cv2, os
    
    # import custom libraries
    import Cubotino_T_set_picamera_gain as camera_set_gains  # script that allows to fix some parameters at picamera
    import Cubotino_T_servos as servo                      # custom library controlling Cubotino servos and led module
    import Cubotino_T_moves as rm                          # custom library, traslates the cuber solution string in robot movements string

    # import non-custom libraries
    from picamera.array import PiRGBArray                  # Raspberry pi specific package for the camera, using numpy array
    from picamera import PiCamera                          # Raspberry pi specific package for the camera
    from statistics import median                          # median is used as sanity check while evaluating facelets contours
    import RPi.GPIO as GPIO                                # import RPi GPIO library
    import datetime as dt                                  # mainly used as timestamp, like on data logging
    import numpy as np                                     # data array management
    import math                                            # math package
    import time                                            # time package
    import cv2                                             # computer vision package
    import os                                              # os is imported to ensure the file presence check/make
    
    print(f'CV2 version: {cv2.__version__}')               # print to terminal the cv2 version
    
    # Up to here Cubotino logo is shown on display
    disp.show_on_display('LOADING', 'SOLVER', fs1=24, fs2=27)  # feedback is printed to the display
    disp.set_backlight(1)                                      # display backlight is turned on, in case it wasn't

    
    # importing Kociemba solver
    # this import takes some time to be uploaded
    # there are thre import attempts, that considers different solver installation methods
    try:                                                  # attempt
        import solver as sv                               # import Kociemba solver copied in /home/pi/cube
        import cubie as cubie                             # import cubie Kociemba solver library part
        solver_found = True                               # boolean to track no exception on import the copied solver
        if debug:                                         # case debug variable is set true on __main__            
            print('found Kociemba solver copied in active folder')   # feedback is printed to the terminal
    except:                                               # exception is raised if no library in folder or other issues
        solver_found = False                              # boolean to track exception on importing the copied solver
       
    if not solver_found:                                  # case the solver library is not in active folder
        import os.path, pathlib                           # library to check file presence            
        folder = pathlib.Path().resolve()                 # active folder (should be home/pi/cube)  
        fname = os.path.join(folder,'twophase','solver.py')   # active folder + twophase' + solver name
        if os.path.exists(fname):                         # case the solver exists in 'twophase' subfolder
            try:                                          # attempt
                import twophase.solver as sv              # import Kociemba solver copied in twophase sub-folder
                import twophase.cubie as cubie            # import cubie Kociemba solver library part
                solver_found = True                       # boolean to track the  import the copied solver in sub-folder
                if debug:                                 # case debug variable is set true on __main__            
                    print('found Kociemba solver copied in subfolder')  # feedback is printed to the terminal
            except:                                       # exception is raised if no library in folder or other issues
                pass                                      # pass, to keep solver_found = False as it was before
        
    if not solver_found:                                  # case the library is not in folder or twophase' sub-folder
        try:                                              # attempt
            import twophase.solver as sv                  # import Kociemba solver installed in venv
            import twophase.cubie as cubie                # import cubie Kociemba solver library part
            twophase_solver_found = True                  # boolean to track no exception on import the installed solver
            if debug:                                     # case debug variable is set true on __main__
                print('found Kociemba solver installed')  # feedback is printed to the terminal
        except:                                           # exception is raised if no library in venv or other issues
            twophase_solver_found = False                 # boolean to track exception on importing the installed solver
    

    if not solver_found and not twophase_solver_found:    # case no one solver has been imported
        print('\nnot found Kociemba solver')              # feedback is printed to the terminal
        disp.show_on_display('NO SOLVER', 'FOUND', fs1=19, fs2=28) # feedback is printed to the display
        time.sleep(5)
        quit_func(quit_script=True)                       # script is quitted






def press_to_start():
    """ Print PRESS TO START to display, and waits until the touch button is pressed to escape the infinite loop"""
    
    global robot_idle
    
    choice = ''                                # variable to store the user choice (solve or scramble)
    robot_idle = True                          # robot is idling
    disp.set_backlight(1)                      # display backlight is turned on, in case it wasn't
    txt1 = 'PRESS TO'                          # text to print at first row
    txt2 = 'START'                             # text to print at second row
    fs_text2 = 32                              # font size at second row
    disp.show_on_display(txt1, txt2, fs2=fs_text2)   # request user to touch the button to start a cycle
    ref_time = time.time()                     # current time is assigned to ref_time variable
    cubotino_logo = False                      # boolean to alternate text with Cubotino logo on display
    display_time = 1                           # time (in sec) for each display page
    while True:                                # infinite loop
        if time.time() - ref_time >= display_time:   # time is checked
            if not cubotino_logo:              # case cubotino logo boolean is false  
                disp.show_cubotino()           # shows the Cubotino logo on the display, for delay time
                ref_time = time.time()         # current time is assigned to ref_time variable
                cubotino_logo = True           # cubotino logo boolean is set true                           
            else:                              # case cubotino logo boolean is false  
                disp.show_on_display(txt1, txt2, fs2=fs_text2)    # request user to touch the button to start a cycle
                ref_time = time.time()         # current time is assigned to ref_time variable
                cubotino_logo = False          # cubotino logo boolean is set false
            time.sleep(0.05)                   # little sleep time 
            
        if GPIO.input(touch_btn):              # case touch_button is 'pressed'
            servo.cam_led_On(cam_led_bright)   # led on top_cover is switched on as button feedback
            choice = solve_or_scramble()       # function for single/double touch (solve or scramble)
            return choice                      # choice is returned, breaking inner and outer while loop
            

##### addition for faire setup ##########
# it uses two inputs in logic AND to start the robot. These buttons don't do enything else (no scrambling, no time filtering)
        if GPIO.input(touch_btn1_faire) and GPIO.input(touch_btn2_faire):
            # case both the touch buttons on the faire setup are 'pressed'
            return 'solve'                     # 'solve' is returned
#########################################
    








def solve_or_scramble():
    """ Sense for a single or double touches at the touch sensor, within a time limit.
        Returns 'solve' when single touch and 'scramble' when multiple touches.
        If the button is kept pressed longer, the robot can be restaterted or shutted OFF."""
    
    global robot_idle, side
    
    choice = ''                               # variable to store the user choice (solve or scramble)
    state = False                             # boolean state to check flank is set false, 
    max_delay = 0.8                           # max time between touches, to decide if single or multi touches
    extra_time = 1.2                          # extra time to accept a single button press as intention to 'solve' a cube
    last_time = time.time()                   # variable holding the time of the last change to 'pressed' button
    pulse_count = 0                           # pulses counter is initially set to zero
    
    while robot_idle:                         # boolean robot_idle is used to sustain this loop
        new_state = GPIO.input(touch_btn)     # touch button status is assigned (it's ON when pressed)
        if new_state and not state:           # case the touch sensor changes from 'not pressed' to 'pressed'
            pulse_count += 1                  # pulse counter is incremented
            state = True                      # set the state to true
            last_time = time.time()           # updtaes the variable that hold
        elif not new_state:                   # case the touch sensor is 'not pressed'
            state = False                     # state is changed to false
        
        # case the elapsed time is bigger than max time and at least 1 pulse
        if time.time() > (last_time + max_delay) and pulse_count > 0:
            if GPIO.input(touch_btn):         # case button is still pressed
                if pulse_count == 1:          # case the button has been pressed only once
                    if time.time()< last_time + max_delay + extra_time: # case time is within the given extratime
                        servo.cam_led_Off()   # led OFF at Top_cover, to suggest the user to remove the finger
                        choice = 'solve'      # 'solve' is assigned to choice variable
                    else:                     # case elapsed time is more than extra_time
                        servo.cam_led_Off()   # led OFF at Top_cover
                        break                 # while loop is interrupted
                
                elif pulse_count > 1:         # case the button has been pressed multiple times
                    servo.cam_led_Off()       # led OFF at Top_cover
                    break                     # while loop is interrupted

            elif not GPIO.input(touch_btn):   # touch button is NOT pressed
                if pulse_count == 1:          # case pulses equal one, and button not 'pressed'
                    choice = 'solve'          # 'solve' is assigned to choice variable

                elif pulse_count >= 2:        # case pulses are two or more,  and button not 'pressed'
                    servo.cam_led_Off()       # led OFF at Top_cover when scrambling is chosen
                    choice = 'scramble'       # 'scramble' is assigned to choice variable

                else:                         # other cases (button kept pressed long without a release within maxtime)
                    pulse_count = 0           # pulse counter is reset to zero
                    choice = ''               # empty string is assigned to choice variable

                return choice                 # choice is returned
    
    
    
    servo.cam_led_Off()    # led OFF at Top_cover when scrambling is chosen
    robot_idle = False     # robot_idle is set false
    stop_or_quit()         # stop or quit function is called, when the while loop is interrupted
    choice = ''            # empty string is assigned to choice variable
    return choice          # choice is returned





def check_screen_presence():
    """ Checks if a display is connected, eventually via VNC; This is not the little display (SPI) ...."""
    
    import os
    
    if 'DISPLAY' in os.environ:                          # case the environment variable DISPLAY is set to 1 (there is a display)
        if debug:                                        # case debug variable is set true on __main__
            print('display function is availbale')       # feedback is printed to the terminal
        return True                                      # function returns true, meaning there is a screen connected
    else:                                                # case the environment variable DISPLAY is set to 0 (there is NOT a display)
        if debug:                                        # case debug variable is set true on __main__
            print('display function is not availbale')   # feedback is printed to the terminal
        return False                                     # function returns false, meaning there is NOT a screen connected







def webcam():
    """ Set the camera and its resolution."""
    
    global camera, PiRGBArray
    
    # PiCamera is set as camera object, with binning. Framerate range limited to 10 allows longer exposure time (when needed)
    # higher framerate does not really speed up the cube reading process.
    s_mode = 7                             # sensor mode 7 means 4x4 binning
    camera = PiCamera(sensor_mode=s_mode, framerate_range = (1, 10))  
    
    rawCapture = PiRGBArray(camera)        # returns (uncoded) RGB array from the camera
    # camera_width_res = 640   #(AF 640)   # PiCamera width resolution setting (other possible values: 800 or 1088)
    # camera_hight_res = 480   #(AF 480)   # PiCamera height resolution setting (other possible values: 544 or 720)
    
    width = camera_width_res               # image width
    height = camera_hight_res              # image height
    camera.resolution = (width, height)    # camera's resolution is set
    time.sleep(0.15)                       # little delay to let the camera setting
    
    binning = camera.sensor_mode           # PiCamera sensor_mode is checked
    time.sleep(0.15)                       # little delay to let the camera setting
    
    if debug:                                      # case debug variable is set true on __main__
        #  sensor_mode answer from the camera is interpreted
        if binning <=3:                            # case the binning is <=3 it means no binning
            binning='none'                         # binning variable is set to none
        elif binning == 4 or binning == 5:         # case the binning is 4 or 5 it means binning 2x2
            binning='2x2'                          # binning variable is set to 2x2
        elif binning > 5:                          # case binning is bigger than 5 it means binning 4x4
            binning='4x4'                          # binning variable is set to 4x4
        print('PiCamera mode (binning):', binning) # feedback is printed to the terminal
        
    if side == 0 and not robot_stop:               # case cube side is zero and no request to stop the robot
        print(f'PiCamera resolution (width x height): {camera.resolution}')  # feedback is printed to the terminal
    
    return camera, rawCapture, width, height







def robot_camera_warmup(camera, start_time):
    """ According to PiCamera documentation it is required a couple of seconds to warmup the camera.
    If the robot start-button is pressed right after placing the cube, the camera has to update the gains previously set,
    that means from when the camera was pointing the black cube support; this can take up to 20 secs to get all
    the gains stable (awb gains are rather slow to update).
    Much different is when the cube is on the cube support for few secs when the button is pressed.
    To properly cover all the possible situations, this fuction releases the camera warm-up phase only after
    all the gains are stable, meaning an absolute variation < 2% from the average of last 2 seconds."""
    
    
    if not robot_stop:
        disp.show_on_display('CAMERA', 'SETUP', fs1=25, fs2=28)  # feedback is printed to the display
        print('\nPiCamera: waiting for AWB and Exposure gains to get stable')  # feedback is printed to the terminal
        if debug:                              # case debug variable is set true on __main__
            print('camera set in auto mode')   # feedback is printed to the terminal
        camera.exposure_mode = 'auto' # set to auto exposure at the start, to adjust according to light conditions
        time.sleep(0.15)              # not found documentation if a delay is needed after this PiCamera setting 
        camera.awb_mode = 'auto'      # set to auto white balance at the start, to adjust according to light conditions
        time.sleep(0.15)              # not found documentation if a delay is needed after this PiCamera setting
        camera.shutter_speed = 0      # set to shutter speed to auto at the start, to adjust according to light conditions
        time.sleep(0.15)              # not found documentation if a delay is needed after this PiCamera setting
        
        a_gain_list=[]                # list to store the Picamera analog gain, during warmup period
        d_gain_list=[]                # list to store the Picamera digital gain, during warmup period
        awb_blue_list=[]              # list to store the Picamera AWB gain, for blue, during warmup period
        awb_red_list=[]               # list to store the Picamera AWB gain, for red, during warmup period
        exp_list=[]                   # list to store the Picamera exposure time, during warmup period
        t_list=[]                     # list to store the warmup progressive time of checkings
        
        # kl = 0.95     #(AF 0.95)    # lower koefficient to define acceptance bandwidth (95%)
        ku = 2-kl                     # Upper koefficient to define acceptance bandwidth (105%)
        pts = 8                       # amount of consecutive datapoints to analyse if parameters within acceptable range
        
        t_start=time.time()           # time reference is assigned as reference for the next task
        
        while time.time()-t_start <20:       # timeout for camera warm-up phase is 20 seconds
            if robot_stop:                   # case the robot has been requested to stop
                PiCamera_param=()            # empty tuple is assigned to PiCamera_param variable
                return PiCamera_param        # PiCamera_param is returned
                
            frame, w, h = read_camera()                                 # camera start reading the cube, and adjusts the awb/exposure
            a_gain=camera.analog_gain                                   # analog gain is inquired to the PiCamera
            d_gain=camera.digital_gain                                  # digital gain is inquired to the PiCamera
            awb_gains=camera.awb_gains                                  # awb blue and red gains are inquired to the PiCamera
            exposure=camera.exposure_speed                              # exposure is inquired to the PiCamera
            
            a_gain_list.append(round(float(a_gain),2))                  # analog gain is appended to a list
            d_gain_list.append(round(float(d_gain),2))                  # digital gain is appended to a list
            awb_blue_list.append(round(float(awb_gains[0]),2))          # awb blue part gain is appended to a list
            awb_red_list.append(round(float(awb_gains[1]),2))           # awb blue part gain is appended to a list
            exp_list.append(exposure)                                   # exposure time (micro secs) is appended to a list
            t_list.append(round(time.time()- t_start,2))                # time (from AWB and Exposure start adjustement) is appended to a list
            
            a_check=False                                               # a flag is negatively set for analog gain (being stable...)
            d_check=False                                               # a flag is negatively set for digital gain (being stable...)
            awb_blue_check=False                                        # a flag is negatively set for awb blue gain (being stable...)
            awb_red_check=False                                         # a flag is negatively set for awb red gain (being stable...)
            exp_check=False                                             # a flag is negatively set for the exposure time (being stable...)
            
            if len(a_gain_list) > pts:                                  # requested a minimum amount of datapoints
                check = a_gain_list[-1]/(sum(a_gain_list[-pts:])/pts)   # last analog gain value is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundaries
                    a_check=True                                        # flag is positively set for this gain

                check = d_gain_list[-1]/(sum(d_gain_list[-pts:])/pts)   # last digital gain value is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundaries
                    d_check=True                                        # flag is positively set for this gain               
            
                check = awb_blue_list[-1]/(sum(awb_blue_list[-pts:])/pts) # last awb_blue gain is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundaries
                    awb_blue_check=True                                 # flag is positively set for this gain    
                    
                check = awb_red_list[-1]/(sum(awb_red_list[-pts:])/pts) # last awb_red gain is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundarie
                    awb_red_check=True                                  # flag is positively set for this gain
                
                check = exp_list[-1]/(sum(exp_list[-pts:])/pts)         # last exposure time is compared to the average of last pts points
                if check > kl and check < ku:                           # if comparison within acceptance boundarie
                    exp_check=True                                      # flag is positively set for the exposure time                
         
                if a_check and d_check and awb_blue_check and awb_red_check and exp_check: # if all gains are stable
                    PiCamera_param=(a_gain, d_gain, awb_gains, exposure)  # latest parameters returned by the PiCamera are assigned to a tuple
                    break                                                 # camera warmup while loop cab be break

            if not robot_stop and screen:            # case screen variable is set true on __main__
                if fixWindPos:                       # case the fixWindPos variable is chosen
                    cv2.namedWindow('cube')          # create the cube window
                    cv2.moveWindow('cube', 0,0)      # move the window to (0,0)
                cv2.imshow('cube', frame)            # shows the frame 
                cv2.waitKey(1)                       # refresh time is minimized to 1ms  
        
        
        print(f'PiCamera: AWB and Exposure being stable in {round(t_list[-1],1)} secs')
        if screen:                                        # case screen variable is set true on __main__
            if debug:                                     # case debug variable is set true on __main__
                print('\nPiCamera warmup function:')      # feedback is printed to the terminal
                print('analog_gain_list',a_gain_list)     # feedback is printed to the terminal
                print('digital_gain_list',d_gain_list)    # feedback is printed to the terminal
                print('awb_blue_list',awb_blue_list)      # feedback is printed to the terminal
                print('awb_red_list',awb_red_list)        # feedback is printed to the terminal
                print('camera exp_list', exp_list)        # feedback is printed to the terminal
                print('time:', t_list)                    # feedback is printed to the terminal
                print('datapoints:', len(t_list))         # feedback is printed to the terminal
        
        disp.clean_display()                              # cleans the display
    
    return PiCamera_param







def robot_consistent_camera_images(camera, PiCamera_param, start_time):
    """ Picamera is left in Auto mode for Exposure and AWB, untill the first 9 facelets of first side are detected.
    For consistent color detection, on the following sides, these two parameters are retrieved from PiCamera right after
    the first side detection, and se back to PiCamera as 'manual' parametes.
    This prevents the PiCamera to keep adjusting the AWB and Exposure while reading the following 5 cube faces (sides)."""
    
    if robot_stop:        # case the robot has been requested to stop
        return            # function is terminated
    
    disp.show_on_display('MEASURING', 'EXPOSURE', fs1=18, fs2=20)    # feedback is printed to the display
    
    a_gain = PiCamera_param[0]                          # analog gain picamera setting when warmup got stable
    d_gain = PiCamera_param[1]                          # digital gainpicamera setting when warmup got stable
    awb_gains = PiCamera_param[2]                       # AutoWhiteBalance gain picamera setting when warmup got stable
    
    if not robot_stop:                                  # case of no request to stop the robot
        print('\nPiCamera: Reading the exposure time on four cube sides')  # feedback is printed to the terminal
        if debug:                                       # case debug variable is set true on __main__
            print('camera awb set off')                 # feedback is printed to the terminal
    
    camera.awb_mode = 'off'                             # sets white balance off
    time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera
    camera.awb_gains = awb_gains                        # sets AWB gain to PiCamera, for consinsent images 
    time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera
    camera_set_gains.set_analog_gain(camera, a_gain)    # sets analog gain to PiCamera, for consinsent images
    time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera
    camera_set_gains.set_digital_gain(camera, d_gain)   # sets digital gain to PiCamera, for consinsent images
    time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera
    camera.shutter_speed = 0      # set the shutter speed to auto at the start, to adjust according to light conditions
    time.sleep(0.1)                                     # small (arbitrary) delay after setting a new parameter to PiCamera

    
    # PiCamera exposure is inquired to PiCaera on 4 cube sides reachable via a simple cube flip, to later fix an average exposure time
    exp_list=[]                                         # list to store the Picamera exposure time, for the first 4 faces
    exp_list.append(camera.exposure_speed)              # read picamera exposure time (microsec) on first face
    for i in range(3):                                  # iterate over the 4 faces reachable via a single cube flip
        if not robot_stop and screen:                   # case screen variable is set true on __main__
            frame, w, h = read_camera()                 # camera start reading the cube, and adjusts the awb/exposure
            if fixWindPos:                              # case the fixWindPos variable is chosen
                cv2.namedWindow('cube')                 # create the cube window
                cv2.moveWindow('cube', 0,0)             # move the window to (0,0)
            cv2.imshow('cube', frame)                   # shows the frame 
            cv2.waitKey(1)                              # refresh time is minimized to 1ms 
        
        robot_to_cube_side(1,cam_led_bright)            # flipping the cube, to reach the next side 
        time.sleep(0.3)                                 # small (arbitrary) delay before reading the exposure time at PiCamera
        exp_list.append(camera.exposure_speed)          # read picamera exposure time (microsec) on the face i
        
    robot_to_cube_side(1,cam_led_bright)                # flipping the cube, to reach the next side
    shutter_time = int(sum(exp_list)/len(exp_list))     # set the shutter time to the average exposure time
    camera.shutter_speed = shutter_time                 # sets the shutter time to the PiCamera, for consinstent images
    time.sleep(0.15)
 
    print(f'Exposure time measured on 4 cube sides, in: {round(time.time()-start_time,1)} secs')# feedback is printed to the terminal
    
    if debug:                                                   # case debug variable is set true on __main__
        print('\nexposure time on UBDF faces : ', exp_list)     # feedback is printed to the terminal
        print('\naverage exposure time: ', int(sum(exp_list)/len(exp_list)), 'micro secs') # feedback is printed to the terminal
        print('shutter_speed time set by PiCamera: ',camera.shutter_speed, ' micro secs')    # feedback is printed to the terminal
        print('\nPiCamera parameters, for consistent images, are set to:')                   # feedback is printed to the terminal
        print('analog_gain:',round(float(a_gain),2))      # feedback is printed to the terminal
        print('digital_gain:',round(float(d_gain),2))     # feedback is printed to the terminal
        print('awb_mode:', camera.awb_mode)               # feedback is printed to the terminal
        print('awb_blue:',round(float(awb_gains[0]),2))   # feedback is printed to the terminal
        print('awb_red:',round(float(awb_gains[1]),2))    # feedback is printed to the terminal
        print(f'camera shutter_speed (0=auto): {shutter_time} micro secs, as average measured on UBDF sides') # feedback is printed to the terminal
    
    disp.clean_display()   # cleans the display






def read_camera():
    """ Returns the camera reading, and dimensions."""
    
    global previous_time

    camera.capture(rawCapture, format='bgr')                      # bgr is the picamera format directly compatible with CV2
    frame = rawCapture.array                                      # picamera array allows usgae of numpy array
    if len(frame)==0:                                             # case the frame is empty
        print('Webcam frame not available')                       # feedback is print to the terminal
    
    else:                                                         # case the frame is not empty
        frame, w, h = frame_cropping(frame, width, height)        # frame is cropped in order to limit the image area to analyze
        frame, w, h = warp_image(frame, w, h)                     # frame is warped to have a top like view toward the top cube face
        frame, w, h = frame_resize(frame, w, h, scale=0.8)        # frame is resized (to smaller size), to gain some speed
        if fps:                                                   # case the fps is requested at main function
            frame = add_fps(frame, w, h)                          # prints the fps on the frame
        oneframe = True                                           # flag for a single frame analysis at the time
        rawCapture.truncate(0)                                    # empties the array in between each camera's capture      

        return frame, w, h





            
def frame_cropping(frame, width, height):
    """Frame cropping, to prevent reading the back cube side and to increase overal speed.
    Due to short camera distance from the cube, all the PiCamera sensor area is used,
    therefore pixels reduction is beneficial ro reduce overall memory load.
    For proper setting of these parameters, it is convenient to temporary skip the frame_warping:
    uncomment first row at frame_warping function."""
    
#     uncomment below row (return frame, width, height) to prevent the image cropping
#     this is useful for initial camera positioning (angle) on the top_cover
#     return frame, width, height


#     x_l = 0 #(AF 60)     # pixels to remove from the frame left side, to reduce memory load 
#     x_r = 0 #(AF 80)     # pixels to remove from the frame right side, to reduce memory load
#     y_u = 0 #(AF 40)     # pixels to remove from the frame top side, to reduce memory load
#     y_b = 0 #(AF 110)    # pixels to remove from the frame bottom side, to reduce memory load
    
    frame = frame[y_u: height-y_b , x_l: width-x_r]   # frame is sliced
    w = width - x_l - x_r                             # sliced frame width
    h = height - y_u - y_b                            # sliced frame height
    
    return frame, w, h







def warp_image(frame, w, h):
    """ Warp the image to remove perspective and simulate a top like view.
    This because PiCamera is at an angle with reference to the top cube face.
    This allows to analyse the facelets contours (square like, area, etc) in a more precise way.
    This also allows a much nicer cube faces collage."""
    
#     NOTE: uncomment below raw to remove the image warping, useful for initial setting the camera frame cropping (frame_cropping)
#     return frame, w, h
    
    if cv_wow:                        # case the cv image analysis plot is set true
        global pre_warp, after_warp   # global variable for the frame content, before and after warping
        pre_warp = frame              # frame is assigned to the pre_warp variable
        ww = w                        # frame width before warping is assigned to a local variable
        hh = h                        # frame height before warping is assigned to a local variable
    
    grid_vertices = np.float32([[0,0], [h,0], [h,w], [0,w]])  # original frame vertices
#     warp_fraction = 7           #(AF 7) fractional frame width part to be 'removed' on top left and right of the frame
    d_x = int(w/warp_fraction)  # pixels to 'remove' on top left and top righ sides of frame, to warp the image
    straight = 1+d_x/h          # corrects the cube face deformation 

    warped_vertices = np.float32([[d_x,0], [h,0], [int(straight*h),w], [-d_x, w]])       # frame coordinates for the transformation matrix
    matrix = cv2.getPerspectiveTransform(warped_vertices, grid_vertices)                 # compute perspective matrix

    # do perspective transformation, by adding black pixels where needed
    frame = cv2.warpPerspective(frame, matrix, (max(w,h), max(w,h)), cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, borderValue=(0,0,0))
    
#     warp_slicing = 1.5                   #(AF 1.5)  # Fractional part of the warping portion to be used as slicing from frame bottom
    frame = frame[: -d_x, :-int(d_x/warp_slicing)]   # frame slicing to remove part of the added (black) pixels on the right frame side 
    h, w = frame.shape[:2]                           # new frame height and width
    
    if cv_wow:                                           # case the cv image analysis plot is set true
        pre_warp, _, _ = frame_resize(pre_warp, ww, hh)  # re-sized frame is assigned to the pre_warp global variable
        after_warp, _, _ = frame_resize(frame, w, h)     # re-sized frame is assigned to the after_warp global variable
    
    return frame, w, h







def frame_resize(frame_w, ww, hh, scale=0.8):        
    """ Re-sizes the image after the cropping and warping steps, by a scaling factor.
    This is useful to lower the amount of handled data for the facelects and color detection."""
    
    ww = int(ww * scale)      # new frame width
    hh = int(hh * scale)      # new frame height
    frame_s = cv2.resize(frame_w, (ww, hh), interpolation = cv2.INTER_AREA)  # resized frame

    return frame_s, ww, hh







def add_fps(frame, w, h):
    """ Number of fps are printed on top_left corner of the frame.
    This function works correctly when"""
    
    global previous_time, fps_i, fps_dict
    
    if fps_i==0:                                           # case fps index is zero, meaning at the start
        fps_dict={}                                        # empty dict is created, to store the fps values
        fps_i=1                                            # fps index is increased to one
        previous_time=time.time()                          # current time is assigned to previous_time variable
        return 0                                           # returns fps zero
        
    else:                                                  # case fps index is not zero, meaning after the start
        fps_value=round(1/(time.time()-previous_time),1)   # fps istant value is calculated
        fps_dict[fps_i]=fps_value                          # dictionary is populated
        n=10                                               # number of values to keep at the dictionary
        if fps_i>n:                                        # case the dict index is bigger than the number of wanted values
            del fps_dict[fps_i-n]                          # the first dict value is removed

        fps=round(sum(fps_dict.values())/len(fps_dict),1)  # fps is calculated as average value
           
        if fps>0.0:                                        # case the fps is bigger than zero (never at the start)
            frame=cv2.copyMakeBorder(frame, 0, 30, 0, 0, cv2.BORDER_CONSTANT, None, (0,0,0))   # adds a black bandwidth at frame bottom
            cv2.putText(frame, str(fps), (10, h+20), font, fontScale, fontColor, lineType)     # adds a text with the fps
        
        previous_time=time.time()                          # current time is assigned to previous_time variable
        fps_i+=1                                           # fps index is increased
        
        return frame







def edge_analysis(frame, w, h):
    """ Image analysis that returns a black & white image, based on the colors borders.
        From 30th July 2022 differentiated the analysis for cube with /withouth the black frame around the facelets."""
    
    if cv_wow and screen:                                    # case screen and cv_wow variables are set true on __main__
        global gray, blurred, canny, dilated, eroded         # images are set as global variable

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)           # from BGR color space to gray scale
    
    if frameless_cube == 'false':                            # case the cube has black frame around the facelets
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)          # low pass gaussian filter, with a 5x5 gaussian filter (9x9 until 30th July 2022)
        canny = cv2.Canny(blurred, 10, 30)                   # single pixel edges, with intensity gradient range 10 to 30
        kernel = np.ones((5,5), np.uint8)                    # kernel of 5x5 pixels for the dilate transformation
        dilated = cv2.dilate(canny, kernel, iterations = 4)  # higher "iterations" is overall faster
        kernel = np.ones((3,3), np.uint8)                    # smaller kernel is used for the erosion
        eroded = cv2.erode(dilated, kernel, iterations = 2)  # smaller "iterations" keeps the contour apart from the edges
    
    elif frameless_cube == 'true':                           # case the cube is a frameless cube
        blurred = cv2.bilateralFilter(gray, 3, 80, 80)       # low pass bilateral filter, to de-noise while safegarding edges
        canny = cv2.Canny(blurred, 4, 25)                    # single pixel edges, with intensity gradient range 4 to 25
        kernel = np.ones((7,7), np.uint8)                    # kernel of 7x7 pixels for the dilate transformation
        dilated = cv2.dilate(canny, kernel, iterations = 4)  # higher "iterations" is overall faster
        kernel = np.ones((5,5), np.uint8)                    # smaller kernel is used for the erosion
        eroded = cv2.erode(dilated, kernel, iterations = 1)  # smaller "iterations" keeps the contour apart from the edges
    
    # note: when frameless_cube == 'auto' the cube detection takes slightly longer
    elif frameless_cube == 'auto':                           # case for cubes with and without the black frame around the facelets
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)          # low pass gaussian filter, with a 5x5 gaussian filter
        canny_01 = cv2.Canny(blurred, 10, 30)                # single pixel edges, with intensity gradient range 10 to 30
        blurred = cv2.bilateralFilter(gray,4, 200, 200)  # 3, 80, 80)        # low pass bilateral filter, to de-noise while safegarding edges
        canny_02 = cv2.Canny(blurred, 4, 25)                 # single pixel edges, with intensity gradient range 4 to 25
        canny = cv2.bitwise_or(canny_01, canny_02, mask = None) # canny image, by (OR) combining those generated with parameters with and without frames
        kernel = np.ones((7,7), np.uint8)                    # kernel of 7x75 pixels for the dilate transformation
        dilated = cv2.dilate(canny, kernel, iterations = 3)  # higher "iterations" is overall faster
        kernel = np.ones((3,3), np.uint8)                    # smaller kernel is used for the erosion
        eroded = cv2.erode(dilated, kernel, iterations = 2)  # smaller "iterations" keeps the contour apart from the edges
    
############ visual check edge detection #############
#     cv2.imshow("Gray", gray)           # gray is shown, on a window called Gray
#     cv2.imshow("blurred", blurred)     # blurred is shown, on a window called Blurred
#     cv2.imshow("Canny", canny)         # canny is shown, on a window called Canny
#     cv2.imshow("Dilated", dilated)     # dilated is shown, on a window called Dilated
#     cv2.imshow("Eroded", eroded)       # eroded is shown, on a window called Eroded
######################################################
    
    return eroded, w, h







def show_cv_wow(cube, time=2000):
    """ shows how the image from the camera is altered to detect the facelets.
        Also possible to set a boolean to save those images."""
    
    gap_w = 40       # horizontal gap (width)
    gap_h = 100      # vertical gap (height)
    
    # note: for precise window position, after windows name at cv2.namedWindow, the parameter cv2.WINDOW_NORMAL  
    # should be used instead of the cv2.WINDOW_RESIZE (default), but image quality on screen changes too much....
    cv2.namedWindow('Pre_warp')                      # create the Pre_warp window
    cv2.moveWindow('Pre_warp', w, gap_h)             # move the Pre_warp window to coordinate
    cv2.namedWindow('After_warp')                    # create the Frame window
    cv2.moveWindow('After_warp', 2*w, gap_h)         # move the Frame window to coordinate
    cv2.namedWindow('Gray')                          # create the Gray window
    cv2.moveWindow('Gray', 3*w, gap_h)               # move the Gray window to coordinate
    cv2.namedWindow('blurred')                       # create the Blurred window
    cv2.moveWindow('blurred', 4*w, gap_h)            # move the Blurred window to coordinate
    cv2.namedWindow('Canny')                         # create the Canny window
    cv2.moveWindow('Canny', w, h+2*gap_h)            # move the Canny window to coordinate
    cv2.namedWindow('Dilated')                       # create the Dilated window
    cv2.moveWindow('Dilated', 2*w, h+2*gap_h)        # move the Dilated window to coordinate
    cv2.namedWindow('Eroded')                        # create the Eroded window
    cv2.moveWindow('Eroded', 3*w, h+2*gap_h)         # move the Eroded window to coordinate
    cv2.namedWindow('Cube')                          # create the Cube window
    cv2.moveWindow('Cube', 4*w, h+2*gap_h)           # move the Cube window to coordinate
    
    cv2.imshow("Pre_warp", pre_warp)                 # pre-warp is shown, on a window called Pre_warp
    cv2.imshow("After_warp", after_warp)             # after_warp is shown, on a window called After_warp
    cv2.imshow("Gray", gray)                         # gray is shown, on a window called Gray
    cv2.imshow("blurred", blurred)                   # blurred is shown, on a window called Blurred
    cv2.imshow("Canny", canny)                       # canny is shown, on a window called Canny
    cv2.imshow("Dilated", dilated)                   # dilated is shown, on a window called Dilated
    cv2.imshow("Eroded", eroded)                     # eroded is shown, on a window called Eroded
    cv2.imshow("Cube", cube)                         # cube is shown, on a window called Cube
    
    cv2.waitKey(time)    # windows show time (ms), for longer time increase timeout variable at start_up()
    
    save_images = False                                         # boolean to enable/disable saving cv_wow images
    folder = pathlib.Path().resolve()                           # active folder (should be home/pi/cube)  
    if save_images:                                             # case the save_image is True
        folder = os.path.join(folder,'cv_wow_pictures')         # folder to store the cv_wow pictures
        if not os.path.exists(folder):                          # if case the folder does not exist
            os.makedirs(folder)                                 # folder is made if it doesn't exist
        datetime = dt.datetime.now().strftime('%Y%m%d_%H%M%S')  # date_time variable is assigned, for file name
        for i, image in enumerate(('Pre_warp', 'After_warp', 'Gray', 'Blurred', 'Canny', 'Dilated', 'Eroded', 'Cube')):
            if side == 1 and i == 0:                                       # case for first image to save
                print('\n\n##########################################')    # feedback is printed to the terminal
                print('###   saving 48 cv_wow pictures  !!!  ####')        # feedback is printed to the terminal
                print('##########################################\n\n')    # feedback is printed to the terminal
            fname = datetime + '_Side'+ str(side) + '_' + str(i) + '_'+ image + '.png'  # filename constructor
            fname = os.path.join(folder, fname)         # folder+filename for the cube data
            if image == 'Pre_warp':                     # case the image equals to Pre_warp
                cv2.imwrite(fname, pre_warp)            # pre_warp image is saved
            elif image == 'After_warp':                 # case the image equals to Frame
                cv2.imwrite(fname, after_warp)          # frameimage is saved
            elif image == 'Gray':                       # case the image equals to Gray
                cv2.imwrite(fname, gray)                # gray image is saved
            elif image == 'Blurred':                    # case the image equals to Blurred
                cv2.imwrite(fname, blurred)             # blurred image is saved
            elif image == 'Canny':                      # case the image equals to Canny
                cv2.imwrite(fname, canny)               # canny image is saved
            elif image == 'Dilated':                    # case the image equals to Dilated
                cv2.imwrite(fname, dilated)             # dilated image is saved
            elif image == 'Eroded':                     # case the image equals to Eroded
                cv2.imwrite(fname, eroded)              # eroded image is saved
            elif image == 'Cube':                       # case the image equals to Cube
                cv2.imwrite(fname, cube)                # cube image is saved
            else:                                       # case the does not match proposed strings
                print('error in image list')            # feedback is printed to the terminal








def square_check(data):  
    """Sanity check if a contour has a square like shape; Argument is a contour
    Calculates quadrilateral's edge delta lenght: [(max edge - min edge)/average of sides length]
    Calculates the ratio between the 2 diagonals (rhonbus axes): min diagonal / max diagonal
    These parameter are later used to verify if the contour can be considered like a square.
    A perfect square will have edge_delta==0 and axes_delta==1."""
    
    edges=[]                    # list of the 4 edges of the quadrilateral
    axes=[]                     # List of axes of symmetry length of the rhombus
    for i in range(len(data)):  # iteration over the contour vertices 
        j = i + 1               # following vertex
        if i==3:                # case the current iteration is on vertex 
            j=0                 # following vertex is set as the first one
        edges.append(math.sqrt((data[j][0]-data[i][0])**2 + (data[j][1]-data[i][1])**2))  # list of the 4 edge's length
        edge_delta = (max(edges)-min(edges))*4/sum(edges)                                 # max side delta over the mean
    
    for i in range(2):          # iteration over 2 (diagonals)
        j = i + 2               # following vertices is the after the next 
        axes.append(math.sqrt((data[j][0]-data[i][0])**2 + (data[j][1]-data[i][1])**2))  # list of the 2 rhobus axes
        axes_delta = min(axes)/max(axes)    # axes delta is the division of the smallest over the largest
    
    return edge_delta, axes_delta







def distance_deviation(data, delta=0.3):
    """ Checks whether the distances between the 9 contours centers are within a pre-defined deviation from the median
    This is a sanity check if all the 9 facelet are compatible with a 3x3 square array shape
    Aim of this funtion is to exclude contours generated outside the cube, on eventual square like shapes
    detected by the camera at the cube background.
    The approach checks indipendently the 6 horizontal distances from the contours center, from the 6 vertical ones.
    Considering the cube can have a certain tilting angle (inclination), Pitagora theorem is used
    Function return a list with the index of the countours to be removed, if excess of distance, from the
    list of contours considered being potential facelets.
    -------------
    | 0 | 1 | 2 |
    -------------
    | 3 | 4 | 5 |
    -------------
    | 6 | 7 | 8 |
    -------------    
    """
    
    d_to_exclude = []        # list of the contour index to be removed, due to excess of distance deviation
    distance_list_h = []     # list of the horizontal distance, of each contour from the median one
    distance_list_v = []     # list of the vertical distance, of each contour from the median one
    
    points_h=[1,2,4,5,7,8]   # coordinates, defining a segment with a previous coordinate,for 6 'horizontal' distances
    for i in points_h:       # iteration over the 6 coordinates
        j=i-1                # the other coordinate used for the segment length
        # horizontal distance between the contours centers
        dist=math.sqrt((data[i]['cx']-data[j]['cx'])**2 + (data[i]['cy']-data[j]['cy'])**2)
        distance_list_h.append(dist)  # list with horizontal distance between the contours centers

    points_v=[3,4,5,6,7,8]   # coordinates, defining a segment with a previous coordinate,for 6 'vertical' distances
    for i in points_v:       # iteration over the 6 coordinates
        k=i-3                # the other coordinate used for the segment length
        # vertical distance between the contours centers
        dist=math.sqrt((data[i]['cx']-data[k]['cx'])**2 + (data[i]['cy']-data[k]['cy'])**2)
        distance_list_v.append(dist)  # list with vertical distance between the contours centers

    dist_median_h = median(distance_list_h)                           # median value for horiz distances
    for i in range(len(distance_list_h)):                             # iteration over the list with the (expected 6) horizontal distance
        delta_dist_h=(distance_list_h[i]-dist_median_h)/dist_median_h # deviation in horiz distance
        if delta_dist_h > delta:                                      # filter if horiz deviation > threshold
            d_to_exclude.append(i) # list with contours indexto exlude, due excess on horiz deviation from median

    dist_median_v = median(distance_list_v)                           # median value for vert distances
    for i in range(len(distance_list_v)):                             # iteration over the list with the (expected 6) vertical distance
        delta_dist_v=(distance_list_v[i]-dist_median_v)/dist_median_v # deviation in vert distance
        if delta_dist_v > delta and i not in d_to_exclude: # filter if horiz deviation > threshold
            d_to_exclude.append(i)  # list with contours indexto exlude, due excess on vert deviation from median
    
    return d_to_exclude






def read_facelets(frame, w, h):
    """ Function that uses cv2 to retrieve contours, from an image (called frame in this case)
    Contours are searched on the 'eroded edges' frame copy
    ROI (Region Of Interest) restricts the image to where the cube images really is

    Notes on 'cv2 find contours'
    Contour's tree is used (cv2.RETR_TREE), to identify children contours (contours within other contrours)
    Approximation (v2.CHAIN_APPROX_SIMPLE) reduces the amount of pixel down to only vertes."""
    
    global prev_side
 
    if side!=prev_side:                           # case the current side differs from the previous side
        if debug:                                 # case debug variable is set true on __main__
            print()                               # print an empty line to the terminal
        print(f'Reading side {sides[side]}')      # feedback is printed to the terminal
        prev_side=side                            # current side is assigned to previous side variable

    image, w, h = edge_analysis(frame, w, h)      # image edges analysis is applied to the frame
    if fps:                                       # case the fps is requested
        image = image[:h,:]                       # image (from edge analysis) is sliced to ROI to prevent reading contours on the fps value
    
    (contours, hierarchy) = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # contours are searched on the image
#     (contours, hierarchy) = cv2.findContours(image.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # contours are searched on the image copy
    
    return (contours, hierarchy)                  # contours are returned
    







def get_approx_contours(component):
    """ Function that simplifies contours (from: https://docs.opencv.org/4.5.3/dd/d49/tutorial_py_contour_features.html)
    Argument is a contour, having at least 4 vertex (contours with less than 4 vertex were previously filtered out)
    Returns approximated contours, having 4 vertex.""" 
    
    contour = component[0]
    hierarchy = component[1][2]
    peri = cv2.arcLength(contour, True)
    contour_convex = cv2.convexHull(contour, False)
    contour = cv2.approxPolyDP(contour_convex, 0.1*peri, True)
    
    return contour, hierarchy, len(contour)







def get_facelets(facelets, frame, contour, hierarchy):
    """ Contours are analyzed in order to detect the cube's facelets; Argument are simplified contours.
    Returns contours having square characteristics
    
    [parameter to be square like: Area within limits, limited egde lenght deviation, limited diagonals lenght deviation
    (to prevent rhonbus), limited area variation between the 9 facelets].
    This function is called on each of the cube sides, therefore the return relates to the face under analysis.""" 
    
    min_area = int(0.08*(w*h)/9)    #(AF int(0.08*(w*h)/9))  # min area limit for a single facelet's contour
    max_area = 6*min_area           #(AF 6*min_area)         # max area limit for a single facelet's contour       

#     square_ratio=1       #(AF 1)    # it is like a square when min side/max side < square_ratio (0==perfect square, 1 is rather permissive)
#     rhombus_ratio=0.3    #(AF 0.3)  # considered like a square when diagonal1/diagonal2 > rhombus_ratio (1==perfect, 0.3 is rather permissive)
         
    area = cv2.contourArea(contour)                                       # area of each passed contour is retrieved
    
    if min_area < area < max_area:                                        # filter out too small and too large contours (areas)                                              # that contour isn't  
        contour_squeeze = np.squeeze(contour)                             # flattens out the list of list used by contours
        edges_delta, axes_ratio = square_check(contour_squeeze)           # sanity check on square and ronbhus shapes
        if edges_delta < square_ratio and axes_ratio > rhombus_ratio:     # check if the contour looks like a square
            cont, in_cont, out_cont = order_4points(contour_squeeze, w, h)  # vertex of each contour are ordered CW from top left
            contour_tmp = [cont]                                          # list is made with the ordered detected contour
            cv2.drawContours(frame, contour_tmp, -1, (255, 255, 255), 1)  # a white polyline is drawn on the contour (1 px thickness)
            if debug:                                                     # case debug variable is set true on __main__
                # a white circle is drawn on the 1st vertex (top left), as visual check of proper vertices ordering
                cv2.circle(frame, (contour_tmp[0][0][0],contour_tmp[0][0][1]), 5, (255, 255, 255), -1)
            contour_tmp = [out_cont]                                      # list is made with the ordered outer contour
            cv2.drawContours(frame, contour_tmp, -1, (0, 0, 0), 1)        # a black polyline is drawn on the outer contour (1 px thickness)
            
            M = cv2.moments(contour)                    # the shape moment (center) of the contour is retrieved
            if M['m00']:                                # compute the center of the contour   
                cX = int(M['m10'] / M['m00'])           # X value for the contour center
                cY = int(M['m01'] / M['m00'])           # Y value for the contour center
            
            tmp = {'area': area, 'cx': cX, 'cy': cY, 'contour': contour, 'cont_ordered':in_cont}  # dict with relevant contour info
            facelets.append(tmp)                        # list with the dictionary of the potential facelets contrours
                                
            if len(facelets)>=5:                        # case there are at least 5 potential contours
                a_to_exclude = area_deviation(facelets, min_area, max_area) # function that analyzes facelets area, and list those with high dev from median
                if len(a_to_exclude)>=1:                # case when there are facelets to be excluded, due to too different area from median one
                    a_to_exclude.sort(reverse=True)     # list order is reversed, making easy easy to remove
                    for i in a_to_exclude:              # contour deviating too much on area are removed from list of potential facelets
                        facelets.pop(i)                 # contour deviating too much on area are removed from list of potential facelets
                else:                                   # case there are not facelets to be excluded, due to large area deviation
                    if frameless_cube != 'false':       # case the cube status reading is for frameless cubes or auto (with and without frames)
                        facelets = estimate_facelets(facelets,frame, w, h)  # calls the function to estimate the remaining facelets                                 
    
    return facelets   # list of potential facelet's contour is returned







def facelet_grid_pos(x, y):
    """returns the face facelet number, based on the coordinates of the contour center, and other parameters.
        This is used to map in which facelets a contour has been detected, and which not."""
    
    # Below dict has the face facelets number as value; keys are facelet coordinate expressed as string combination of column and row
    facelet_number = {'11':0, '21':1, '31':2,
                      '12':3, '22':4, '32':5,
                      '13':6, '23':7, '33':8}
    
    facelet = str(x) + str(y)                 # facelet string combining column (x) and row (y) of the facelet
    if facelet in facelet_number:             # case the facelecet is in the dict
        return facelet_number[facelet]        # facelet number is returned







def estimate_facelets(facelets, frame, w, h):
    """Estimates the remaing facelets location, when there are at least 5 detected facelets.
        This function is interrupted if one row or column is fully empty; In this way the cube width and height
        is well known, enabling a good estimation for the missed facelets position.
        This function is called when the frameless_cube is set true or auto.
        This function is not called when the the setting is exclusively set for cubes with black frame around the facelets.
    
    
                x_1     x_2     x_3              X
           --|----------------------------------->
             |       |       |       |
        y_1  |   1   |   2   |   3   |   row 1
             |       |       |       |
             |-----------------------
             |       |       |       |
        y_2  |   4   |   5   |   6   |   row 2
             |       |       |       |
             |----------------------- 
             |       |       |       |
        y_3  |   7   |   8   |   9   |   row 3
             |       |       |       |
             |-----------------------
             |
             | clmn1   clmn2   clmn3
             |
             |
           Y v
         
    """
    
    
    cont_x = []                                # empty list to fill with contours centers x coordinates
    cont_y = []                                # empty list to fill with contours centers y coordinates
    cont_area = []                             # empty list to fill with contours areas
    facelets_detected = len(facelets)          # number of facelets alerady detected
    
    for i in range(facelets_detected):         # iteration over the quantity of facelets already detected
        cont_x.append(facelets[i]['cx'])       # all the contours centers x coordinates are listed
        cont_y.append(facelets[i]['cy'])       # all the contours centers y coordinates are listed
        cont_area.append(facelets[i]['area'])  # all the contours areas are listed
    
    med_a = int(median(cont_area))             # median area for the facelets in function argument
    
    cx = cont_x.copy()         # list copy of contours centers x coordinates
    cy = cont_y.copy()         # list copy of contours centers y coordinates
    cont_x.sort()              # sorted list with contours centers x coordinates
    cont_y.sort()              # sorted list with contours centers y coordinates
    
    x_1 = []                   # empty list to fill with x coordinates of first column (smaller x)
    x_2 = []                   # empty list to fill with x coordinates of second column (medium x)
    x_3 = []                   # empty list to fill with x coordinates of third column (larger x)
    y_1 = []                   # empty list to fill with y coordinates of first row (smaller y)
    y_2 = []                   # empty list to fill with y coordinates of second row (medium y)
    y_3 = []                   # empty list to fill with y coordinates of third row (larger y)

    x_low = cont_x[0]          # smaller x coordinate of facelets countours is assigned to the variable x_low
    x_high = cont_x[-1]        # bigger x coordinate of facelets countours is assigned to the variable x_high
    y_low = cont_y[0]          # smaller y coordinate of facelets countours is assigned to the variable y_low
    y_high = cont_y[-1]        # bigger y coordinate of facelets countours is assigned to the variable y_high 
    
    dist = int(max(x_high-x_low, y_high-y_low)/4) # facelets separation distance from min/max detected contours centers
    
    x_1.append(x_low)          # smaller x coordinate of facelets countours is appended to the first column list
    y_1.append(y_low)          # smaller y coordinate of facelets countours is appended to the first row list
    
    for i in range(1, facelets_detected):                              # iteration on detected facelets contours
        if x_low <= cont_x[i] and cont_x[i] < x_low + dist:            # case the contour center x coordinate is "small"
            x_1.append(cont_x[i])                                      # contour x coordinate is appended to the of first column list (small x)
        elif x_low + dist <= cont_x[i] and cont_x[i] < x_high - dist:  # case the contour center x coordinate is "medium"
            x_2.append(cont_x[i])                                      # contour x coordinate is appended to the of second column list (medium x)
        else:                                                          # case the contour center x coordinate is "large"
            x_3.append(cont_x[i])                                      # contour x coordinate is appended to the of third column list (large x)
        
        if y_low <= cont_y[i] and cont_y[i] < y_low + dist:            # case the contour center y coordinate is "small"
            y_1.append(cont_y[i])                                      # contour y coordinate is appended to the of first row list (small y)
        elif y_low + dist <= cont_y[i] and cont_y[i] < y_high - dist:  # case the contour center y coordinate is "medium"
            y_2.append(cont_y[i])                                      # contour y coordinate is appended to the of second row list (medium y)
        else:                                                          # case the contour center y coordinate is "large"
            y_3.append(cont_y[i])                                      # contour y coordinate is appended to the of third row list (large y)
    
    if len(x_1)==0 or len(x_2)==0 or len(x_3)==0 or len(y_1)==0 or len(y_2)==0 or len(y_3)==0: # case one or more of the six lists are empty
        return facelets                                                # function returns the already detected facelets
    
    else:                                      # case no one of the six lists is empty
        x1_avg = int(sum(x_1)/len(x_1))        # average x coordinate for the contours on first column (small x)
        x2_avg = int(sum(x_2)/len(x_2))        # average x coordinate for the contours on second column (medium x)
        x3_avg = int(sum(x_3)/len(x_3))        # average x coordinate for the contours on third column (large x)
        y1_avg = int(sum(y_1)/len(y_1))        # average y coordinate for the contours on first row (small y)
        y2_avg = int(sum(y_2)/len(y_2))        # average y coordinate for the contours on second row (medium y)
        y3_avg = int(sum(y_3)/len(y_3))        # average y coordinate for the contours on third row (large y)
    
    dist = int((x_high - x_low + y_high - y_low)/8)   # facelets separation distance from min/max detected contours centers
    detected = []                                     # list for the column row of the face grid
    for i in range(facelets_detected):                # iteration over the detected facelets
        if cx[i]<x_low+dist:                          # case the facelet contour center x coordinate is on first grid column
            x=1                                       # 1 (as column 1) is assigned
        elif cx[i]>x_low+dist and cx[i]<x_high-dist:  # case the facelet contour center x coordinate is on second grid column
            x=2                                       # 2 (as column 2) is assigned
        elif cx[i]>x_high-dist:                       # case the facelet contour center x coordinate is on third grid column
            x=3                                       # 3 (as column 3) is assigned
        if cy[i]<y_low+dist:                          # case the facelet contour center x coordinate is on first grid row                          
            y=1                                       # 1 (as row 1) is assigned
        elif cy[i]>y_low+dist and cy[i]<y_high-dist:  # case the facelet contour center x coordinate is on second grid row
            y=2                                       # 2 (as row 2) is assigned
        elif cy[i]>y_high-dist:                       # case the facelet contour center x coordinate is on third grid row
            y=3                                       # 3 (as row 3) is assigned
        detected.append(facelet_grid_pos(x, y))       # list with facelet number is populated
    
    s = set(detected)                                        # list with detected facelets numbers is transformed to set
    missed = [x for x in (0,1,2,3,4,5,6,7,8) if x not in s]  # list with missed facelets numbers
    
    est = []                                  # list for xy coordinates for the estimated facelet center locations
    for facelet in missed:                    # iteration over the missed facelets numbers
        if facelet == 0:                      # case the missed facelet is 0
            est.append((x1_avg, y1_avg))      # average xy coordinates for column 1 and row 1 are appended
        elif facelet == 1:                    # case the missed facelet is 1
            est.append((x2_avg, y1_avg))      # average xy coordinatees for column 2 and row 1 are appended
        elif facelet == 2:                    # case the missed facelet is 2
            est.append((x3_avg, y1_avg))      # average xy coordinats for column 3 and row 1 are appended
        elif facelet == 3:                    # case the missed facelet is 3
            est.append((x1_avg, y2_avg))      # average xy coordinates for column 1 and row 2 are appended
        elif facelet == 4:                    # case the missed facelet is 4
            est.append((x2_avg, y2_avg))      # average xy coordinates for column 2 and row 2 are appended
        elif facelet == 5:                    # case the missed facelet is 5
            est.append((x3_avg, y2_avg))      # average xy coordinates for column 3 and row 2 are appended
        elif facelet == 6:                    # case the missed facelet is 6
            est.append((x1_avg, y3_avg))      # average xy coordinates for column 1 and row 3 are appended
        elif facelet == 7:                    # case the missed facelet is 7
            est.append((x2_avg, y3_avg))      # average xy coordinates for column 2 and row 3 are appended
        elif facelet == 8:                    # case the missed facelet is 8
            est.append((x3_avg, y3_avg))      # average xy coordinates for column 3 and row 3 are appended
        else:                                 # case that shouldn't exist
            print("error on estimating the missed facelets")  # feedback is printed to the terminal

    semi_side = int(0.85*dist/2)                              # half side dimension for the estimated contour square
    for i in range(len(missed)):                              # iteration over the missed facelets
        tl = [est[i][0] - semi_side, est[i][1] - semi_side]   # top left contour coordinate, calculated from the estimated contour center point
        tr = [est[i][0] + semi_side, est[i][1] - semi_side]   # top right contour coordinate, calculated from the estimated contour center point
        br = [est[i][0] + semi_side, est[i][1] + semi_side]   # bottom right contour coordinate, calculated from the estimated contour center point
        bl = [est[i][0] - semi_side, est[i][1] + semi_side]   # bottom left contour coordinate, calculated from the estimated contour center point
        pts=np.array([tl, tr, br, bl], dtype="int32")         # estimated contour coordinates      
        
        gap=5                                  # pixels gap
        tl[0]=max(tl[0]-gap,0)                 # top left x coordinate, shifted toward the contour outer side
        tl[1]=max(tl[1]-gap,0)                 # top left y coordinate, shifted toward the contour outer side
        tr[0]=min(tr[0]+gap,w)                 # top right x coordinate, shifted toward the contour outer side
        tr[1]=max(tr[1]-gap,0)                 # top right y coordinate, shifted toward the contour outer side
        br[0]=min(br[0]+gap,w)                 # bottom right x coordinate, shifted toward the contour outer side
        br[1]=min(br[1]+gap,h)                 # bottom right y coordinate, shifted toward the contour outer side
        bl[0]=max(bl[0]-gap,0)                 # bottom left x coordinate, shifted toward the contour outer side
        bl[1]=min(bl[1]+gap,h)                 # bottom left y coordinate, shifted toward the contour outer side
        outer_pts=np.array([tl, tr, br, bl], dtype="int32")     # estimated contour coordinates, sligtly shifted toward the contour outer side
        
        contour_tmp = [pts]                                           # list is made with the ordered outer contour
        cv2.drawContours(frame, contour_tmp, -1, (0, 0, 0), 1)        # a black polyline is drawn on the contour (1 px thickness)
        contour_tmp = [outer_pts]                                     # list is made with the ordered outer contour
        cv2.drawContours(frame, contour_tmp, -1, (255, 255, 255), 1)  # a white polyline is drawn on the outer contour (1 px thickness)

        tmp = {'area': med_a, 'cx': est[i][0], 'cy': est[i][1], 'contour': pts, 'cont_ordered':pts} # dict with relevant contour info
        facelets.append(tmp)                   # estimated facelets relevant info are appended to the detected facelets list
    
    if screen and debug and frameless_cube!=False:                             # case there is a connected screen, debug and Frameless_cube not False
        cv2.line(frame, (x_low+dist,0), (x_low+dist,h), (255, 255, 255), 2)    # vertical line is drawn between first and second row of facelets
        cv2.line(frame, (x_high-dist,0), (x_high-dist,h), (255, 255, 255), 2)  # vertical line is drawn between second and third row of facelets
        cv2.line(frame, (0,y_low+dist), (w,y_low+dist), (255, 255, 255), 2)    # horizontal line is drawn between first and second column of facelets
        cv2.line(frame, (0,y_high-dist), (w,y_high-dist), (255, 255, 255), 2)  # horizontal line is drawn between second and third column of facelets
        
    return facelets     # detected facelets combined with estimated facelets








def area_deviation(data, min_area, max_area):
    """ Checks whether the areas of 9 facelets are within a pre-defined deviation from the median one
    This function is called when there are a pre-defined amount of potential facelet contours
    Argument is a list of dictionary, wherein the area is one of the dict values
    Returns a list of facelets (index) to be removed from the potential facelets, having area deviating
    too much from the median one."""

    
    to_exclude = []                  # list of the contours index to be removed, due to excess of their area deviation
#     delta_area_limit = 0.7            #(AF 0.7)   # 70% of area deviation from the median is set as threshold (quite permissive)
    area_list = []                   # list to store the contour areas
    
    for i in range(len(data)):
        area_list.append(data[i]['area'])                       # all the contour areas are listed
#         print("contours areas detected:",area_list)             # feedback is printed to the terminal

    area_median = median(area_list)                             # median area values
    for i in range(len(area_list)):                             # iteration over the facelets areas received
        delta_area=abs((area_list[i]-area_median)/area_median)  # area deviation from the median
        if delta_area > delta_area_limit:                       # filter on area deviating more than threshold
            to_exclude.append(i)                                # list of the contours to exclude is populated
            if debug:                                           # case debug variable is set true on __main__
                print('removed contour with area: ',area_list[i], " having delta_area of:",delta_area) # feedback is printed to the terminal

    if debug:                                                   # case debug variable is set true on __main__
        if len(to_exclude)==0 and len(area_list)>=9:            # case all the face facelets have been detected
            if side == 1:                                       # case the face is the first one (U)
                print("acceptable facelets area from min:",min_area, ",  to max:", max_area, ", with max area_delta <=", delta_area_limit)
            print("median facelets area:", area_median, "\tareas min:", min(area_list), "\tareas_max:", max(area_list))  # feedback is printed to the terminal
            print("minimum facelet area delta vs median:", round(abs((min(area_list)-area_median)/area_median),2))       # feedback is printed to the terminal
            print("maximum facelet area delta vs median:", round(abs((max(area_list)-area_median)/area_median),2))       # feedback is printed to the terminal
            print()
                      
    return to_exclude                # returns list of contours to be removed








def order_4points(pts, w, h):
    """ Based on: https://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
    Modified to only use Numby instead also Scipy library.
    This function orders the 4 vertex of (simplified) contours, so that the first one is top left (CW order)
    Argument is a contour
    Function returns a contour with coordinates ordered as per below sketch
    
    0  1
    2  3
    """
    
    xSorted = pts[np.argsort(pts[:, 0]), :]        # sort the points based on their x-coordinates
    leftMost = xSorted[:2, :]                      # grab the left-most point from the sorted x-coodinate points
    rightMost = xSorted[2:, :]                     # grab the right-most point from the sorted x-coodinate points
    
    # sort the left-most according to their y-coordinates, to grab the top-left and bottom-left points, respectively
    (tl, bl) = leftMost[np.argsort(leftMost[:, 1]), :]
    
    # Euclidean distance from top-left and right-most points: the point with largest distance will be bottom-right
    D = tl.reshape(1, 2)[:, np.newaxis, :] - rightMost[np.newaxis, :, :]   # matrix difference, by broadcasting tl and rightMost
    D = np.linalg.norm(D, axis=-1)[0]                                      # matrix distance of all rightMost vertices from tl
    
    (br, tr) = rightMost[np.argsort(D)[::-1], :]
    pts=np.array([tl, tr, br, bl])         # CW ordered coordinates, from top left, of the detected contour
    
    gap=3                                  # pixels for contour shifting toward inside and outside
    tl[0]=min(tl[0]+gap,w)                 # top left x coordinate, shifted toward the contour inner side
    tl[1]=min(tl[1]+gap,h)                 # top left y coordinate, shifted toward the contour inner side
    tr[0]=max(tr[0]-gap,0)                 # top right x coordinate, shifted toward the contour inner side
    tr[1]=min(tr[1]+gap,h)                 # top right y coordinate, shifted toward the contour inner side
    br[0]=max(br[0]-gap,0)                 # bottom right x coordinate, shifted toward the contour inner side
    br[1]=max(br[1]-gap,0)                 # bottom right y coordinate, shifted toward the contour inner side
    bl[0]=min(bl[0]+gap,w)                 # bottom left x coordinate, shifted toward the contour inner side
    bl[1]=max(bl[1]-gap,0)                 # bottom left y coordinate, shifted toward the contour inner side
    inner_pts=np.array([tl, tr, br, bl], dtype="int32")   # ordered coordinates, sligtly shifted toward the contour inner side
    
    gap=2*gap                              # pixels gap is doubled as it now refers to coordinates already shifted toward the inside
    tl[0]=max(tl[0]-gap,0)                 # top left x coordinate, shifted toward the contour outer side
    tl[1]=max(tl[1]-gap,0)                 # top left y coordinate, shifted toward the contour outer side
    tr[0]=min(tr[0]+gap,w)                 # top right x coordinate, shifted toward the contour outer side
    tr[1]=max(tr[1]-gap,0)                 # top right y coordinate, shifted toward the contour outer side
    br[0]=min(br[0]+gap,w)                 # bottom right x coordinate, shifted toward the contour outer side
    br[1]=min(br[1]+gap,h)                 # bottom right y coordinate, shifted toward the contour outer side
    bl[0]=max(bl[0]-gap,0)                 # bottom left x coordinate, shifted toward the contour outer side
    bl[1]=min(bl[1]+gap,h)                 # bottom left y coordinate, shifted toward the contour outer side
    outer_pts=np.array([tl, tr, br, bl], dtype="int32")   # ordered coordinates, sligtly shifted toward the contour outer side

    return pts, inner_pts, outer_pts







def order_9points(data, new_center):
    """ Based on: https://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
    Modified to only use Numby instead also Scipy library.
    This function orders the 9 countorus centers, with first one on top left, and order as per below sketch:
    
    0  1  2
    3  4  5
    6  7  8   """
    
    pts=np.zeros([9,2], dtype=int)                 # numpy array of zeros, having 9 items of two entries
    for i in range(len(data)):                     # iteration over the (9) facelets coordinates
        pts[i]=[data[i]['cx'], data[i]['cy']]      # numpy array is filled with the facelets centers coordinates
        
    xSorted = pts[np.argsort(pts[:, 0]), :]        # sort all the points based on their x-coordinates
    leftMost = xSorted[:3, :]                      # grab the left-most 3 points from the sorted x-coodinate points
    rightMost = xSorted[6:, :]                     # grab the right-most 3 points from the sorted x-coodinate points
    mid = xSorted[3:6, :]                          # remaining 3 points in the x middle
    ySortedMid = mid[np.argsort(mid[:, 1]), :]     # sorts the 3 points in the x middle by the y coordinate
    (tm, mm, bm) = ySortedMid                      # top-middle, middle-middle, bottom-midle points
    
    # sort the 3 left-most points according to their y-coordinates, to grab the top/mid/bottom one respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, ml, bl) = leftMost
    
    # Euclidean distance from top-left and right-most points: the point with largest distance will be bottom-right
    D = tl.reshape(1, 2)[:, np.newaxis, :] - rightMost[np.newaxis, :, :]   # matrix difference, by broadcasting tl and rightMost
    D = np.linalg.norm(D, axis=-1)[0]                                      # matrix distance of all rightMost vertices from tl
    
    # sort the right-most according to their distance from top-left coordinate
    (br, mr, tr) = rightMost[np.argsort(D)[::-1], :]
    
    ordered_points = np.array([tl, tm, tr, ml, mm, mr, bl, bm, br], dtype="int32")   # ordered coordinates (centers of 9 facelets)   
    
    for coordinates in ordered_points:
        for i in range(len(data)):
            if data[i]['cx'] == coordinates[0] and data[i]['cy'] == coordinates[1]:        
                new_center.append(data[i])        # new_center is a new list with data ordered by xy coordinates
                data.pop(i)                       # used data-element is removed to speed up next iterations
                break                             # inner for loop can be break once the if if found and data appended
    return new_center







def cube_colors_interpr(BGR_detected):
    """ This function is used to decide wich color belongs to which facelet (cube's side) and related facelet position
    From the mean BGR color, detected per each facelet, the euclidean distance is calculated toward the 6 reference colors (centers).
    The basic principle is to measure the color distance from the cube's center facelets.
    
    Due to camera vignetting and light conditions, some color interpretation issues were sometimes happening (i.e. swapped red and orange)
    To solve this problem the reference colors are averaged with the just interpreted facelets (dinamic color references)
    In this approach the facelets need to be ordered by increasing distance from the reference; This allows to initially choose the more
    certain facelets, and use them to adapt the references.
    After completing this process, for all the facelets, the interpreted colors have to be ordered back to URFDLB order.
    
    In case the BGR color distance doesn't provide coherent result, a second approach based on HVS color space takes place."""

    # Step1: dict with BGR_detected and facelet's position as key
    #        dict with HSV (detected color) and facelet's position as key
    BGR_detected_dict={}                                    # empty dict to store the average BGR values detected per each facelet
    HSV_detected={}                                         # empty dict to store the average HSV values detected per each facelet
    for i in range(len(BGR_detected)):                      # iteration over the (expected 54) elemnt is list with avg BGR detected 
        BGR=BGR_detected[i]                                 # BGR detected on facelet i
        BGR_detected_dict[i]=BGR                            # BGR detected on facelet i is assigned to the dict, having the facelet i as key
        hsv = cv2.cvtColor( np.array([[[BGR[0],BGR[1],BGR[2]]]], dtype=np.uint8), cv2.COLOR_BGR2HSV)   # HSV color space
        HSV_detected[i]=(hsv[0][0][0],hsv[0][0][1],hsv[0][0][2])                                       # HSV tuple 

    if debug:                                         # case debug variable is set true on __main__
        print(f'\nBGR_detected: {BGR_detected}')      # feedback is printed to the terminal
        print(f'\nHSV: {HSV_detected}')               # feedback is printed to the terminal

    # Step2: detected BGR, of the center's facelets, are used as initial reference
    cube_ref_colors = {'white':BGR_detected[4], 'red':BGR_detected[13], 'green':BGR_detected[22],
                       'yellow':BGR_detected[31], 'orange':BGR_detected[40], 'blue':BGR_detected[49]}
    
    # Step3: dictionary with the color distances from the (initial) references
    color_distance={}                                             # empty dict to store all the color distances for all the facelets
    cube_ref_colors_lab={}                                        # empty dictionary to store color refences in Lab color space
    for color, BGR in cube_ref_colors.items():                    # iteration over the 6 centers
        B,G,R = BGR                                               # BGR values are unpact from the dict
        cube_ref_colors_lab[color]=tuple(rgb2lab([R,G,B]))        # BGR conversion to lab color space and dict feeding
            
    for facelet, color_measured in BGR_detected_dict.items():     # iteration over the 54 facelets
        B,G,R = color_measured                                    # individual BGR components
        lab_meas = rgb2lab([R,G,B])                               # conversion to lab color space (due CIEDE2000 function)
        distance=[]                                               # list with the distance from the 6 references, for each facelet
        for color, lab_ref in cube_ref_colors_lab.items():        # iteration over the 6 reference colors
            distance.append(CIEDE2000(tuple(lab_meas), lab_ref))  # Euclidean distance toward the 6 reference colors
        color_distance[facelet]=distance   # dict (facelet num as key) is populated with the measured distance from the 6 centes
    
    
    # Step4: Ordering the color distance (the min value per each facelet) by increasing values
    color_distance_copy=color_distance.copy()                     # a dict copy is made, to drop items while the analysis progresses
    color_distance_ordered={}                                     # empty dictiony to store the (min) color distance by increasing values
    for i in range(len(color_distance_copy)):                     # iteration over the dict copy
        key_min_dist = min(color_distance_copy,key=lambda key:min(color_distance_copy[key])) # dict key for the facelet with smaller color dist
        color_distance_ordered[key_min_dist] = color_distance_copy[key_min_dist]  # dict with facelets ordered by smaller distance, is populated
        color_distance_copy.pop(key_min_dist)   # removed the facelet from the dic copy, to search the next facelect with smaler color distance
    
    
    # Step5: List with facelets position ordered according to the color distance increasing order
    # this is needed to come back later to original URFDLB facelets order
    key_ordered_by_color_distance = [x for x in color_distance_ordered.keys()]


    # Step6: Ordering the facelets BGR color values according to color distance from the reference colors
    BGR_ordered={}
    for key in color_distance_ordered.keys():
        BGR_ordered[key]=BGR_detected[key]    # key is always the facelet number, yet the ordered dict is by increasing color distance


    # Step7: Color interpretation
    cube_status_by_color_distance={}          # dict to store the cube status reppresentation wih the interpreted colors
    distance={}                               # dict to store the color distance during each facelet check
#     distance_value=[]                       # list to store the color distance for the selectec color/facelet association
                                         
    for i, value in enumerate(BGR_ordered.values()):        # iteration on the facelet's BGR values ordered by increasing color distance from ref
        B,G,R = value
        lab_meas = rgb2lab([R,G,B])                                         # conversion to lab color space (due CIEDE2000 function)
        for color, lab_ref in cube_ref_colors_lab.items():                  # iteration over the 6 reference colors
            distance[color]=CIEDE2000(tuple(lab_meas), lab_ref)             # Euclidean distance toward the 6 reference colors
        color = min(distance, key=distance.get)                             # chosem color is the one with min distance from reference
  
        cube_status_by_color_distance[i]=color                              # dict of cube status wih the interpreted colors  
#         distance_value.append(distance[min(distance, key=distance.get)])  # list with the color distance of the chosen facelet's color
        distance.clear()                                                    # distance dict is cleared for the next facelet
        
        B_avg = math.sqrt((B**2+ (cube_ref_colors[color][0])**2)/2)     # average Red color is made from the chosen color and previous reference
        G_avg = math.sqrt((G**2+ (cube_ref_colors[color][1])**2)/2)     # average Green color is made from the chosen color and previous reference
        R_avg = math.sqrt((R**2+ (cube_ref_colors[color][2])**2)/2)     # average Blue color is made from the chosen color and previous reference

        cube_ref_colors[color]=(B_avg, G_avg, R_avg)                    # Color reference dict is updated with the new BGR averaged color
        cube_ref_colors_lab[color]=tuple(rgb2lab([R_avg,G_avg,B_avg]))  # Lab color space reference dict is updated with the new color reference 
    
    
    # Step8: Cube detection status is generated (a dict having the facelet number as key and the color as value)
    # colors are still as per the conventional one (URFDLB being White Red Green Yellow Orange Blue), later will match to those really detected at URFDLB
    cube_status={}
    for i in range(54):  # iteration over the 54 facelets
        cube_status[i]=cube_status_by_color_distance[key_ordered_by_color_distance.index(i)] # dict with facelet as key and color as value
    
    
    # Step9: Cube color sequence, to plot a realistic cube status detection (by colors, not by URFDLB letters)
    VS_value={}                            # dict to store the V-S (Value-Saturation) value of all facelets
    Hue={}                                 # dict to store the Hue value of all facelets
    
    i=0                                    # iterator index
    for H,S,V in HSV_detected.values():    # Hue, Saturation and Value are retrieved from the HSV dict
        VS_value[i]=int(V)-int(S)          # V-S (value-Saturation) value for all the facelet, populates the related dict
        Hue[i]=int(H)                      # Hue, for all the facelets, populates the related dict
        i+=1                               # iterator index is increased
    
    #step 10: function to get the color (sides) order, list of colored center's facelets and white center facelet
    # this depends on the cube orientation when dropped on the robot
    cube_color_sequence, HSV_analysis = retrieve_cube_color_order(VS_value, Hue)  # call to the specific function
    if debug:                                                   # case debug variable is set true on __main__
        print(f'\nCube_color_sequence: {cube_color_sequence}')  # feedback is printed to the terminal
    
    return cube_status, HSV_detected, cube_color_sequence, HSV_analysis







def retrieve_cube_color_order(VS_value,Hue):
    """ Determines the cube colors order, meaning the cube's orientation as it has been dropped on the robot.
    The function returns a list with the color order, as per URFDLB sequence.
    The function also returns a boolean if the HSV analysis has been coherent."""
    
    HSV_analysis=True                                      # flag used to validate the analysis
    
    centers=[4, 13, 22, 31, 40, 49]                        # facelets of the cube's sides centers 
    cube_color_sequence=[4, 13, 22, 31, 40, 49]            # list of center facelets, to later store the cube's side COLORS
    
    if debug:                                              # case debug variable is set true on __main__
        print(f'\nHue centers: {Hue[centers[0]]}, {Hue[centers[1]]},'\
              f'{Hue[centers[2]]}, {Hue[centers[3]]}, {Hue[centers[4]]}, {Hue[centers[5]]}') # feedback is printed to the terminal
    
    VS_centers=[VS_value[facelet] for facelet in centers]  # V-S value measured on the cube side center under analysis
    Hcenters=[Hue[facelet] for facelet in centers]         # Hue value measured on the cube side center under analysis
    
    
    # white and yellow facelet face (center side facelet number)
    white_center=centers[VS_centers.index(max(VS_centers))]   # white center facelet (max V-S distinguishes the white)
    if white_center<27:                       # case the facelet id is < 27  
        yellow_center=white_center+27         # yellow center facelet (yellow is at opposite side of white, in this case requires adding 27)
    else:                                     # case the facelet id is > 27  
        yellow_center=white_center-27         # yellow is at opposite side of white, in this case by subtrcting 27
    
    try:
        centers.remove(white_center)          # white facelet is removed from the list of center's facelets
    except:                                   # exception is raised if there are more white centers, in case of a bad color detection
        HSV_analysis=False                    # analysis according to HSV is set False
        if debug:                             # case debug variable is set true on __main__
            print(f'\nIssue with the white_center')   # feedback is printed to the terminal
    
    try:
        centers.remove(yellow_center)         # yellow facelet is removed from the list of center's facelets
    except:                                   # exception is raised if there are more yellow centers, in case of a bad color detection
        HSV_analysis=False                    # analysis according to HSV is set False
        if debug:                             # case debug variable is set true on __main__
            print(f'\nIssue with the yellow_center')    # feedback is printed to the terminal
       
    
    # searching for the red and orange cube's side
    for facelet in centers:                   # iteration on the 4 cube's remaining sides centers
        if facelet<27:                        # case the facelet id is < 27         
            opp_facelet=facelet+27            # opposite side center facelet, by adding 27
        elif facelet>27:                      # case the facelet id is > 27   
            opp_facelet=facelet-27            # opposite side center facelet, by subtracting 27
        Hc=Hue[facelet]                       # Hue value measured on the cube side center under analysis
        H_opp=Hue[opp_facelet]                # Hue value measured on the cube opposite side center

        if (Hc>150 and H_opp<30) or (Hc<30 and H_opp<30 and Hc<H_opp) or (Hc>160 and H_opp>170 and Hc<H_opp): # Hue filter for red vs orange
            red_center=facelet                # red center facelet
        elif (Hc<30 and H_opp>150) or (Hc<30 and Hc>H_opp) or (Hc>170 and Hc>H_opp):   # Hue filter for orange vs red
            orange_center=facelet             # orange center facelet
    
    try:
        centers.remove(red_center)            # red facelet is removed from the list of center's facelets
    except:                                   # exception is raised if there are more red centers, in case of a bad color detection
        HSV_analysis=False                    # analysis according to HSV is set False
        if debug:                                   # case debug variable is set true on __main__
            print(f'\nIssue with the red_center')   # feedback is printed to the terminal
    
    try:
        centers.remove(orange_center)         # orange facelet is removed from the list of center's facelets
    except:                                   # exception is raised if there are more orange centers, in case of a bad color detection
        HSV_analysis=False                    # analysis according to HSV is set False
        if debug:                             # case debug variable is set true on __main__
            print(f'\nIssue with the orange_center')   # feedback is printed to the terminal

################ DEBUG  #########
#     HSV_analysis=False    # uncoment this row to force an error on the 6 center facelets color detection
#################################
    
    if HSV_analysis==True:                        # case the HSV_analysis is true
        # last two colors are blue and green, wherein blue has the highest Hue
        Hc=[Hue[facelet] for facelet in centers]  # Hue value for the last two center's facelets
        blue_center=centers[Hc.index(max(Hc))]    # blue center facelet (higher Hue than green)
        green_center=centers[Hc.index(min(Hc))]   # green center facelet (lower Hue than blue)
        
        
        for index, value in enumerate(cube_color_sequence): # iteration over the 6 centers
            if value == white_center:                       # case the value (facelet id) equals to 'white' id, when URFDLB
                cube_color_sequence[index] = 'white'        # 'white' is assigned to the dict, with key the face order according to URFDLB
            elif value == yellow_center:                    # case the value (facelet id) equals to 'yellow' id, when URFDLB
                cube_color_sequence[index] = 'yellow'       # 'yellow' is assigned to the dict, with key the face order according to URFDLB
            elif value == red_center:                       # case the value (facelet id) equals to 'red' id, when URFDLB
                cube_color_sequence[index] = 'red'          # 'red' is assigned to the dict, with key the face order according to URFDLB
            elif value == orange_center:                    # case the value (facelet id) equals to 'orange' id, when URFDLB
                cube_color_sequence[index] = 'orange'       # 'orange' is assigned to the dict, with key the face order according to URFDLB
            elif value == blue_center:                      # case the value (facelet id) equals to 'blue' id, when URFDLB
                cube_color_sequence[index] = 'blue'         # 'blues' is assigned to the dict, with key the face order according to URFDLB
            elif value == green_center:                     # case the value (facelet id) equals to 'green' id, when URFDLB
                cube_color_sequence[index] = 'green'        # 'green' is assigned to the dict, with key the face order according to URFDLB
    else:                                                                            # case the HSV_analysis is false
        print('\nNot found 6 different colors at center facelets (HSV analysis)')    # feedback is printed to the terminal
        
    return cube_color_sequence, HSV_analysis







def cube_colors_interpr_HSV(BGR_detected, HSV_detected):
    """ In case the color interpretation, based on BGR color distance, doesn't give coherent cube status, this function is called.
    Under bright/sunny light condition the colors, chosen via color distance analysys, are sometimes wrongly interpreted
    (=not coherent cube status);
    This function make use on the HSV color space, converted from the avg BGR read on facelets.
    This funcion re-determines the colors assigned to all facelets, based on HSV value measured.
    Baseline:  White has higest V (Value) and lowest S (Saturation) than the other colors; Used "(V-S)" parameter
               Not-white facelets are evaluated only by their H (Hue) value
    Hue (range 0 to 180) average values:  Orange 6, Yellow 32, Green 85, Blue 110, Red 175 (also < 10 when it overflows)
    Note: Depending on the light conditions could be 0< Red_Hue <10 ; In these cases Orange_Hue > Red_Hue
    Note: In some extreme cases orange_hue >175; In these cases Orange_Hue > Red_Hue
    
    This function returns the interpreted facelet colors, on the URFDLB order.
    The function also returns the cube color sequence, to correctly plot the cube status on the cube faces collage.""" 
    
    print('cube_colors_interpr_HSV function has been called')   # feedback is printed to the terminal
    
    VS_value={}                             # dict to store the V-S (Value-Saturation) value of all facelets
    Hue={}                                  # dict to store the Hue value of all facelets
    
    i=0                                           # iterator index
    for H,S,V in HSV_detected.values():           # Hue, Saturation and Value are retrieved from the HSV dict
        VS_value[i]=int(V)-int(S)                 # V-S (value-Saturation) delta value for all the facelet, populates the related dict
        Hue[i]=int(H)                             # Hue, for all the facelets, populates the related dict
        i+=1                                      # iterator index is increased
    if debug:                                     # case debug variable is set true on __main__
        print(f'\nV-S_values: {VS_value}')        # feedback is printed to the terminal
        print(f'\nHue: {Hue}')                    # feedback is printed to the terminal
    
    # function to get the color (sides) order, and list of the colored center's facelets plus the white center facelet  
    cube_color_sequence, HSV_analysis = retrieve_cube_color_order(VS_value,Hue)    
    
    while HSV_analysis == True:                   # flag on positive analysis is placed on true, and later eventually changed
        
        # getting the white center's facelet and the colored facelets
        centers=[4, 13, 22, 31, 40, 49]                              # center facelet numbers
        white_center = centers[cube_color_sequence.index('white')]   # facelet number for the white center
        centers.remove(white_center)                                 # white facelet center is removed from the list of facelets centers
        colored_centers=centers                                      # colored centers (without the white)
        
        if debug:                                                    # case debug variable is set true on __main__
            print(f'\nWhite facelet number: {white_center}')         # feedback is printed to the terminal
            print(f'\nCube_color_sequence: {cube_color_sequence}')   # feedback is printed to the terminal
        
        # searching all the 9 white facelets
        Hw,Sw,Vw=HSV_detected[white_center]                              # HSV of the white center
        if debug:                                                        # case debug variable is set true on __main__
            print(f'White Hw,Sw,Vw: {Hw}, {Sw}, {Vw}')                   # feedback is printed to the terminal
            print(f'Colored center facelets numbers: {colored_centers}') # feedback is printed to the terminal


        VSdelta={}                                                       # dict to store the V-S (value-Saturation) value of all facelets
        i=0                                                              # iterator index
        for H,S,V in HSV_detected.values():                              # Hue, Saturation and Value are retrieved from the HSV dict
            VSdelta[i]=int((V)-int(S))                                   # difference between value (brightness) and saturation, for all the facelets
            i+=1                                                         # iterator index is increased
        
        # ordering the VSdelta by increasing values (9 highest values are the 9 white facelets)
        VSdelta_copy=VSdelta.copy()       # a dict copy is made, to drop items while the analysis progresses
    
        # a new dict with V-S delta value ordered is generated, in order to have the white facelets close to each other
        VSdelta_ordered={k: v for k, v in sorted(VSdelta_copy.items(), key=lambda item: item[1])}
        key_ordered_by_VSdelta = [x for x in VSdelta_ordered.keys()]    # list with the key of the (ordered) dict is generated
        white_facelets_list=key_ordered_by_VSdelta[-9:]                 # white facelets have the biggest H-S value, therefore are the last 9

        if debug:                                                       # case debug variable is set true on __main__
            print(f'White facelets: {white_facelets_list}')             # feedback is printed to the terminal
            print(f'\nHue dict all facelets {Hue}')                     # feedback is printed to the terminal
        
        white_facelets={}  # empty dict to store the association of facelet position for the white facelets
        
        # white facelets are removed from Hue dictionary, as meant for colored facelets
        for facelet in white_facelets_list:                # iteration on all the white facelets listed
            try:
                del Hue[facelet]                           # all white facelets are removed from the Hue dictionary
                white_facelets[facelet]='white'            # dictionary with white facelets is populated
            except:                                        # except is raised if there weren't 6 white facelets
                HSV_analysis = False                       # HSV_analysis is set false
                print('issue at cube_colors_interpr_HSV function')  # feedback is printed to the terminal
                break                                      # for loop is interrupted
        
        if debug:                                                 # case debug variable is set true on __main__
            print(f'\nHue dict colored facelets {Hue}')           # feedback is printed to the terminal
        
        # facelet's color selection by Hue distance from each center's Hue
        centers=[4,13,22,31,40,49]                                # facelets of centers
        centers.remove(white_center)                              # facelets for colored centers
        cube_color_sequence_no_white=cube_color_sequence.copy()   # cube color sequence copy for list without white
        cube_color_sequence_no_white.remove('white')              # cube color sequence, for colored centers
        
        try:
            Hcenters=[Hue[x] for x in colored_centers]            # list of Hue values for the colored centers
            
        except:                                                   # an exception is raised if the same color is on more centers
            if debug:                                             # case debug variable is set true on __main__
                print('\nNot found 6 different colors at center facelets')  # feedback is printed to the terminal
            HSV_analysis=False                                    # HSV_analysis is set false
            break                                                 # for loop is interrupted
        
        if debug:                                                 # case debug variable is set true on __main__
            print(f'\nHcenters (no white): {Hcenters}')           # feedback is printed to the terminal
        
        Hc_red=Hue[colored_centers[cube_color_sequence_no_white.index('red')]]     # Hue value for the red center facelet
        Hc_blue=Hue[colored_centers[cube_color_sequence_no_white.index('blue')]]   # Hue value for the blue center facelet
        red_blue_avg=(Hc_red+Hc_blue)//2                                           # mid Hue value between red and blue
        
        # red is supposed to have Hue >160, yet sometime it is below 10  (Hue ranges 0 to 180 in cv2, and it might overflow)
        # distance from the Hue of center facelets is used to decide the facelet's color
        color_facelets={}                                           # dict to store all the colored facelets (NO WHITE)
        for facelet, H in Hue.items():                              # iteration on all the color facelets
            Hdistance=[]                                            # list to store the Hue distance from the 5 colored centers                   
            if sum(value>red_blue_avg for value in Hue.values())<5: # condition suggesting some of the red facelets have Hue < 10
                for Hc in Hcenters:                                 # iteration over the 5 reference Hue colors
                    if H>red_blue_avg:                              # when the facelet's Hue is above mid distance between Red center and Blue center
                        H=0                                         # Hue is force to 0
                    if Hc>red_blue_avg:                             # Red center Hue (when the center facelet is above mid distance between Red center and Blue )
                        Hc=0                                        # Red center Hue is forced to 0
                    dist=min(abs((H-Hc)),180-Hc+H)                  # red can either be 170<Hue<180 as well as <10
                    Hdistance.append(dist)                          # absolute Hue distance toward the 5 colored centers Hue
            else:                                                   # else case is when all the 6 red's facelts Hue are above mid Hue distance blue / red
                for Hc in Hcenters:                                 # iteration over the 5 reference Hue colors
                    dist=min(abs((H-Hc)),180-Hc+H)                  # red can either be 170<Hue<180 as well as <10
                    Hdistance.append(dist)                          # absolute Hue distance toward the 5 colored centers Hue

            color=cube_color_sequence_no_white[Hdistance.index(min(Hdistance))] # right color has min distance from center facelet of same color
            color_facelets[facelet]=color                                       # color is assigned to dictionary


        # Merging the white and color facelets dictionaries, and sorting it by key
        cube_status_detected={**white_facelets, **color_facelets}                          # dict with white and colored facelets
        cube_status_detected={k: v for k, v in sorted(cube_status_detected.items(), key=lambda item: item[0])}   # dict with white & color ordered by facelets

        # cube can be positioned on the robot with an orientationon different from conventional URFDLB order
        std_cube_color=['white', 'red', 'green', 'yellow', 'orange', 'blue']               # conventional color sequence
        if cube_color_sequence != std_cube_color:                                          # case the cube not oriented on the robot as per std URFDLB color
            cube_status_URFDLB={}                                                          # dict with cube status using conventional colors
            for i, color in enumerate(cube_status_detected.values()):                      # iteration on dict with detected colors
                cube_status_URFDLB[i]=std_cube_color[cube_color_sequence.index(color)]     # colors are exchanged, and assigned to the dict for solver
            if debug:                                                                      # case debug variable is set true on __main__
                print(f'\nCube_status_detected_colors: {cube_status_detected}')            # feedback is printed to the terminal
                print(f'\nCube_status_conventional_colors: {cube_status_URFDLB}\n')        # feedback is printed to the terminal
            break    # this break quits the while loop (HSV_analysis == True), as the analysis is finally completed 
            
        elif cube_color_sequence == std_cube_color:                                        # case the cube is oriented on the robot as per std URFDLB color
            cube_status_URFDLB = cube_status_detected                                      # detected cube staus is assigned to the URFDLB one
            if debug:                                                                      # case debug variable is set true on __main__
                print('\nCube colors and orientation according to URFDLB order')           # feedback is printed to the terminal
                print(f'\nCube_status_detected_colors: {cube_status_detected}\n')          # feedback is printed to the terminal
            break    # this break quits the while loop (HSV_analysis == True), as the analysis is finally completed 
    
    if HSV_analysis==False:      # case the HSV_analysis is false
        cube_status_URFDLB={}    # an empty dict is assigned to the cube_status_URFDLB variable
        cube_status_detected={}  # an empty dict is assigned to the cube_status_detected variable
        
    # cube_status_URFDLB uses 'conventional color sequence' (URFDLB being White Red Green Yellow Orange Blue), convenient to later build the cube_status
    # cube_status_detected has the detected colors, via the HSV approach. This dict is used for decoration purpose
    return cube_status_URFDLB, cube_status_detected, cube_color_sequence






def rgb2lab(inputColor):
    """ Convert RGB (not BGR !!!) in L*a*b colors space
    from: https://gist.github.com/manojpandey/f5ece715132c572c80421febebaf66ae (RGB to CIELab color space conversion)
        Step 1: RGB to XYZ
                http://www.easyrgb.com/index.php?X=MATH&H=02#text2
        Step 2: XYZ to Lab
                http://www.easyrgb.com/index.php?X=MATH&H=07#text7
    
    L*a*b color space is a device-independent, "standard observer" model, is useful in industry for detecting small differences in color."""
    
    num = 0
    RGB = [0, 0, 0]
    for value in inputColor:
        value = float(value) / 255
        if value > 0.04045:
            value = ((value + 0.055) / 1.055) ** 2.4
        else:
            value = value / 12.92
        RGB[num] = value * 100
        num = num + 1
    XYZ = [0, 0, 0, ]
    X = RGB[0] * 0.4124 + RGB[1] * 0.3576 + RGB[2] * 0.1805
    Y = RGB[0] * 0.2126 + RGB[1] * 0.7152 + RGB[2] * 0.0722
    Z = RGB[0] * 0.0193 + RGB[1] * 0.1192 + RGB[2] * 0.9505
    XYZ[0] = round(X, 4)
    XYZ[1] = round(Y, 4)
    XYZ[2] = round(Z, 4)

    # Observer= 2°, Illuminant= D65
    XYZ[0] = float(XYZ[0]) / 95.047         # ref_X =  95.047
    XYZ[1] = float(XYZ[1]) / 100.0          # ref_Y = 100.000
    XYZ[2] = float(XYZ[2]) / 108.883        # ref_Z = 108.883

    num = 0
    for value in XYZ:
        if value > 0.008856:
            value = value ** (0.3333333333333333)
        else:
            value = (7.787 * value) + (16 / 116)
        XYZ[num] = value
        num = num + 1
    Lab = [0, 0, 0]
    L = (116 * XYZ[1]) - 16
    a = 500 * (XYZ[0] - XYZ[1])
    b = 200 * (XYZ[1] - XYZ[2])

    Lab[0] = round(L, 4)
    Lab[1] = round(a, 4)
    Lab[2] = round(b, 4)
    return Lab







def CIEDE2000(Lab_1, Lab_2):
    """ Calculates CIEDE2000 color distance between two CIE L*a*b* colors
    from: https://github.com/lovro-i/CIEDE2000
    It returns the Euclidean distance between two colors, and it is used to compare each facelet toward the 6 centers."""
    
    C_25_7 = 6103515625 # 25**7

    L1, a1, b1 = Lab_1[0], Lab_1[1], Lab_1[2]
    L2, a2, b2 = Lab_2[0], Lab_2[1], Lab_2[2]
    C1 = math.sqrt(a1**2 + b1**2)
    C2 = math.sqrt(a2**2 + b2**2)
    C_ave = (C1 + C2) / 2
    G = 0.5 * (1 - math.sqrt(C_ave**7 / (C_ave**7 + C_25_7)))
    
    L1_, L2_ = L1, L2
    a1_, a2_ = (1 + G) * a1, (1 + G) * a2
    b1_, b2_ = b1, b2
    
    C1_ = math.sqrt(a1_**2 + b1_**2)
    C2_ = math.sqrt(a2_**2 + b2_**2)
    
    if b1_ == 0 and a1_ == 0: h1_ = 0
    elif a1_ >= 0: h1_ = math.atan2(b1_, a1_)
    else: h1_ = math.atan2(b1_, a1_) + 2 * math.pi
    
    if b2_ == 0 and a2_ == 0: h2_ = 0
    elif a2_ >= 0: h2_ = math.atan2(b2_, a2_)
    else: h2_ = math.atan2(b2_, a2_) + 2 * math.pi

    dL_ = L2_ - L1_
    dC_ = C2_ - C1_    
    dh_ = h2_ - h1_
    if C1_ * C2_ == 0: dh_ = 0
    elif dh_ > math.pi: dh_ -= 2 * math.pi
    elif dh_ < -math.pi: dh_ += 2 * math.pi        
    dH_ = 2 * math.sqrt(C1_ * C2_) * math.sin(dh_ / 2)
    
    L_ave = (L1_ + L2_) / 2
    C_ave = (C1_ + C2_) / 2
    
    _dh = abs(h1_ - h2_)
    _sh = h1_ + h2_
    C1C2 = C1_ * C2_
    
    if _dh <= math.pi and C1C2 != 0: h_ave = (h1_ + h2_) / 2
    elif _dh  > math.pi and _sh < 2 * math.pi and C1C2 != 0: h_ave = (h1_ + h2_) / 2 + math.pi
    elif _dh  > math.pi and _sh >= 2 * math.pi and C1C2 != 0: h_ave = (h1_ + h2_) / 2 - math.pi 
    else: h_ave = h1_ + h2_
    
    T = 1-0.17*math.cos(h_ave-math.pi/6)+0.24*math.cos(2*h_ave)+0.32*math.cos(3*h_ave+math.pi/30)-0.2*math.cos(4*h_ave-63*math.pi/180)
    
    h_ave_deg = h_ave * 180 / math.pi
    if h_ave_deg < 0: h_ave_deg += 360
    elif h_ave_deg > 360: h_ave_deg -= 360
    dTheta = 30 * math.exp(-(((h_ave_deg - 275) / 25)**2))
    
    R_C = 2 * math.sqrt(C_ave**7 / (C_ave**7 + C_25_7))  
    S_C = 1 + 0.045 * C_ave
    S_H = 1 + 0.015 * C_ave * T
    
    Lm50s = (L_ave - 50)**2
    S_L = 1 + 0.015 * Lm50s / math.sqrt(20 + Lm50s)
    R_T = -math.sin(dTheta * math.pi / 90) * R_C

    k_L, k_C, k_H = 1, 1, 1
    
    f_L = dL_ / k_L / S_L
    f_C = dC_ / k_C / S_C
    f_H = dH_ / k_H / S_H
    
    dE_00 = math.sqrt(f_L**2 + f_C**2 + f_H**2 + R_T * f_C * f_H)
    
    return dE_00







def URFDLB_facelets_order(BGR_mean):
    """ Orders the facelet's colors (BGR values) according to the URFDLB order.
    When the robot is used, faces are detected according to a convenient (robot) order.
    Argument of this function is a list with the BGR mean values (9 facelets at the time), detected by the camera while
    following the detection order at the robot.
    Function returns a dict with the BGR mean values of the 54 facelets, ordered as per URFDLB order.
    The returned dict values are therefore ordered as requested by the kociemba solver."""
    
    robot_facelets_BGR_mean = BGR_mean.copy()                        # (list) copy of detected BGR mean values
    
    robot_sides_order = ['U', 'B', 'D', 'F', 'R', 'L']               # robot sides order when detecting facelets color
    URFDLB_sides_order = ['U', 'R', 'F', 'D', 'L', 'B' ]             # URFDLB conventional sides order
    
    robot_facelets_list = [x for x in range(54)]                     # list of the facelets id, ordered from 0 to 53
    robot_facelets_order = {}                                        # dictiorary of facelets values per side (key)
    for side in robot_sides_order:                                   # iteration over the 6 facelets centers colors, as per robot detection order
        robot_facelets_order[side] = robot_facelets_list[:9]         # dictiorary of facelets values per side (key)
        robot_facelets_list = robot_facelets_list[9:]                # remaining facelets to be assigned to following sides
    for i in range(54-len(BGR_mean)):                                # iteration over the facelets that are not yet detected
        robot_facelets_BGR_mean.append((230,230,230))                # gray facelets are added on facelets not yet detected
    
    URFDLB_facelets_list=[]                                          # list of facelets collected to be properly filles
    URFDLB_facelets_order={}                                         # dictiorary of facelets values per side (key)
    
    for side in URFDLB_sides_order:                                  # iteration over the 6 facelets centers colors, with URFDLB order
        URFDLB_facelets_order[side] = robot_facelets_order[side]     # cross reference robot to URFDLB
        for facelets in robot_facelets_order[side]:                  # facelets id for the face (side) detected by the robot
            URFDLB_facelets_list.append(facelets)                    # facelets id are appended to the the URFDLB list of facelets
    URFDLB_facelets_BGR_mean=[]                                      # empty list to store the BGR average color of all the facelets

    for facelet in URFDLB_facelets_list:                             # iteration over the 54 faceltes
        #BGR tuples ordered as URFDLB facelets order (colors are progressively changed from gray to the detected ones)
        URFDLB_facelets_BGR_mean.append(robot_facelets_BGR_mean[facelet])
    
    return URFDLB_facelets_BGR_mean







def cube_string(cube_status):
    """ Generates the cube detected status string, compatible with the solver:
    All uppercase letters indicating the color's initial
    Argument is the cube status generated, whein the values are the facelet colors (full color name)."""
    
    cube_in_letters = {'white':'U', 'red':'R', 'green': 'F', 'yellow':'D', 'orange':'L', 'blue':'B'}  
    string=''
    for color in cube_status.values():         # iteration over the 54 dict facelets values
        string+=str(cube_in_letters[color])    # string is made with the facelet letter on the side
    return string







def scrambling_cube():
    """function to scramble the cube via the robot.
        The function first generate a random cube status, via a function available form the Kociemba solver package.
        After, the robot generates the moves to solve that specific cube status."""
    
    global robot_idle  
    
    robot_idle = False              # robot is not anymore idling
    disp.show_on_display('CUBE', 'SCRAMBLING', fs1=28, fs2=17)  # feedback is printed on the display
    time.sleep(1)                   # little delay, to let user reading the screen
    start_time = time.time()        # current time 
    cc = cubie.CubieCube()          # cube in cubie reppresentation
    cc.randomize()                  # randomized cube in cubie reppresentation 
    random_cube_string = str(cc.to_facelet_cube())   # randomized cube in facelets string reppresentation
    print("Random cube status:", random_cube_string) # feedback is printed to the terminal
    solution, solution_Text = cube_solution(random_cube_string, scrambling = True) # Kociemba solver is called to have the solution string
    print(solution_Text)            # feedback is printed to the terminal
    
    # dict and string with robot movements, and total movements
    _, robot_moves, total_robot_moves = rm.robot_required_moves(solution, solution_Text)
    print(f'Total robot movements: {total_robot_moves}')  # nice information to print at terminal
    if not robot_stop:              # case there are no request to stop the robot
        robot_move_cube(robot_moves, total_robot_moves, solution_Text, start_time, scrambling=True) # movements to the robot are finally applied
    
    if not robot_stop:              # case there are not request to stop the robot
        servo.read()                # top cover positioned to have the PiCamera in read position
        servo.cam_led_test()        # the led on top_cover is shortly activated once the scrambilng is done







def cube_solution(cube_string, scrambling=False):
    """ Calls the Hegbert Kociemba solver, and returns the solution's moves
    from: https://github.com/hkociemba/RubiksCube-TwophaseSolver 
    (Solve Rubik's Cube in less than 20 moves on average with Python)
    The returned string is slightly manipulated to have the moves amount at the start."""
    
    if not scrambling:
        disp.show_on_display('SOLUTION', 'SEARCH', fs1=21, fs2=27)
#     sv_max_moves = 20     #(AF 20)  # solver parameter: max 20 moves or best at timeout
#     sv_max_time = 2       #(AF 2)   # solver parameter: timeout of 2 seconds, if not solution within max moves
    s = sv.solve(cube_string, sv_max_moves, sv_max_time)  # solver is called
    
#################  solveto function to reach a wanted cube target from a known starting cube status   ######
#     cube_sts = 'UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB'
#     cube_target = 'UDUDUDUDURLRLRLRLRFBFBFBFBFDUDUDUDUDLRLRLRLRLBFBFBFBFB'
#     s = sv.solveto(cube_sts,cube_target, sv_max_moves, sv_max_time)  # solver is called
#     print('moves to cube target',cube_target)
#############################################################################################################
    
    solution = s[:s.find('(')]      # solution capture the sequence of manoeuvres
    
    # solution_text places the amount of moves first, and the solution (sequence of manoeuvres) afterward
    solution_Text = s[s.find('(')+1:s.find(')')-1]+' moves  '+ s[:s.find('(')] 
    if solution[:5] =='Error':        # solution could start with "Error" in case of incoherent cusbe string sent to the solver
        solution_Text = 'Error'       # in that case a short error string is returned
    
    return solution, solution_Text







def decoration(deco_info):
    """ Plots the cube's status made by a collage of images taken along the facelets color detection
    On the collage is also proposed the cube's sketches made with detected and interpreted colors
    This picture collage is saved as image, by adding date and time on the file name as reference."""
    
    # retrieve the variables from function arg (tuple)
    fixWindPos = deco_info[0]
    screen = deco_info[1]
    frame = deco_info[2]
    faces = deco_info[3]
    cube_status = deco_info[4]
    cube_color_sequence = deco_info[5]
    HSV_analysis = deco_info[6]
    cube_status_string = deco_info[7]
    URFDLB_facelets_BGR_mean = deco_info[8]
    color_detection_winner = deco_info[9]
    show_time = deco_info[10]
    timestamp = deco_info[11]


    # collage function is called
    collage=faces_collage(faces, cube_status, color_detection_winner, cube_color_sequence, HSV_analysis, cube_status_string, \
                          URFDLB_facelets_BGR_mean, font, fontScale, lineType)   # call the function that makes the pictures collage
    
    folder = pathlib.Path().resolve()                    # active folder (should be home/pi/cube)  
    folder = os.path.join(folder,'CubesStatusPictures')  # folder to store the collage pictures

    if not os.path.exists(folder):                       # if case the folder does not exist
        os.makedirs(folder)                              # folder is made if it doesn't exist
    fname = folder+'/cube_collage'+timestamp+'.png'      # folder+filename with timestamp for the resume picture
    if debug:                                            # case debug variable is set true on __main__
        print('unfolded cube status image is saved : ', fname) # feedback is printed to the terminal
    status=cv2.imwrite(fname, collage)                   # cube sketch with detected and interpred colors is saved as image
    
    if not robot_stop and screen:                        # case screen variable is set true on __main__
        cv2.namedWindow('cube_collage')                  # create the collage window
        cv2.moveWindow('cube_collage', 0,0)              # move the collage window to (0,0)
        cv2.imshow('cube_collage', collage)              # while the robot solves the cube the starting status is shown
        cv2.waitKey(int(show_time*1000))                 # showtime is trasformed to seconds 
        try:
            cv2.destroyWindow('cube_collage')            # cube_collage window is closed after the show_time
        except:
            pass







def faces_collage(faces, cube_status, color_detection_winner, cube_color_sequence, HSV_analysis, cube_status_string, \
                  URFDLB_facelets_BGR_mean, font, fontScale, lineType):
    """ This function merges multipe pictures, and it returns the unfolded cube single image.
    The 6 cube faces images, taken while detecting the facelets colors, are used for this collage
    The 6 cube faces images are resized to a predefined dimension
    Gray rectangles are generated and used to complete the picture
    Once the collage is made, the original dict of images is cleared, to save some memory."""
    
    face_h = faces[1].shape[0]                                   # height of the cube's face1 image
    face_h = min(face_h,250)                                     # if the cube's face1 image height if bigger than 250, then 250 is chosen  
    for i in range(1,7):                                         # image of all cube faces (1 to 6 )
        faces[i]=cv2.resize(faces[i], (face_h, face_h), interpolation = cv2.INTER_AREA)  # are resized to square of 300 pixels, or face1 height
    empty_face = np.zeros([face_h, face_h, 3],dtype=np.uint8)    # empty array having same dimensions of cube's faces images
    empty_face.fill(230)                                         # array is filled with light gray
    
    # faces[6] is till on memory, 
    resume_h = faces[6].shape[0]            # width of faces[6]
    resume_w = faces[6].shape[1]            # height of faces[6]
    resume_ratio=resume_w/resume_h          # image ratio (w/h) is calculated, as this image differs from the cube's faces images
    resume_h=3*face_h                       # resume image height, to occupy 3 cube's faces
    resume_w=int(3*face_h*resume_ratio)     # resume image width is calculated from the imposed height, by keeping aspect ratio
    
    # cube's faces orientation, under Picamera, are different than the conventional URFDLB order
    # faces number are 1 to 6 according the the scan order (U,B, D, F, F, L)
    for face in [1, 3, 4, 5]:                                      # these faces are 180 rotate from user view standpoint
        faces[face]=cv2.rotate(faces[face],cv2.ROTATE_180)         # these images are then rotated by 180

    seq=[1,5,4,3,6,2]                                              # faces order at robot is hard coded for movements convenience
    col1=np.vstack([empty_face, faces[seq[4]], empty_face])        # vertical stack of images, to generate 1st column for the pictures collage
    col2=np.vstack([faces[seq[0]], faces[seq[2]], faces[seq[3]]])  # vertical stack of images, to generate 2nd column for the pictures collage
    col3=np.vstack([empty_face, faces[seq[1]], empty_face])        # vertical stack of images, to generate 3rd column for the pictures collage
    col4=np.vstack([empty_face, faces[seq[5]], empty_face])        # vertical stack of images, to generate 4th column for the pictures collage
    faces.clear()                                                  # dictionary of images is cleared

    collage = np.hstack([col1,col2,col3,col4])                     # horizzontal stack of 4 columns of images, to generate the pictures collage
    collage_ratio = collage.shape[1] / collage.shape[0]            # collage ratio (width/height) is calculated for resizing
#     collage_w=1024                                    #(AF 1024)   # colleage width is fixed for consistent pictures archiving at Rpi
    collage_h=int(collage_w/collage_ratio)                         # colleage heigth is calculated to maintain proportions
    
    if not light_program:                                          # case the light_program (processor type dependant) is set false
        collage = cv2.resize(collage, (collage_w, collage_h), interpolation = cv2.INTER_AREA) # resized collage
    
    # adds a sketch with interpreted colors (bright) on the collage
    plot_interpreted_colors(cube_status, color_detection_winner, cube_color_sequence, \
                            HSV_analysis, cube_status_string, collage, collage_w, collage_h, \
                            font, fontScale, lineType)     # call the function that updates the cube sketch 
    
    return collage







def cube_sketch_coordinates(x_start, y_start, d):
    """ Generates a list and a dict with the top-left coordinates of each facelet, as per the URFDLB order.
    These coordinates are later used to draw a cube sketch
    The cube sketch (overall a single rectangle with all the facelets in it) starts at x_start, y_start
    Each facelet on the sketch has a square side dimention = edge, defined at start_up() function."""
    
    square_start_pt=[]   # lits of all the top-left vertex coordinate for the 54 facelets
    
    starts={0:(x_start+3*d, y_start), 1:(x_start+6*d, y_start+3*d),
            2:(x_start+3*d, y_start+3*d), 3:(x_start+3*d, y_start+6*d),
            4:(x_start, y_start+3*d), 5:(x_start+9*d, y_start+3*d)}      # dict with the top-left coordinate of each face (not facelets !)
    
    for value in starts.values():                          # iteration over the 6 faces
        x_start=value[0]                                   # x coordinate fo the face top left corner
        y_start=value[1]                                   # y coordinate fo the face top left corner
        y = y_start                                        # y coordinate value for the first 3 facelets
        for i in range(3):                                 # iteration over rows
            x = x_start                                    # x coordinate value for the first facelet
            for j in range(3):                             # iteration over columns
                square_start_pt.append([x, y])             # x and y coordinate, as list, for the top left vertex of the facelet is appendended
                x = x+d                                    # x coordinate is increased by square side
                if j == 2: y = y+d                         # once at the third column the row is incremented
    square_dict = {k:tuple(square_start_pt[k]) for k in range(len(square_start_pt))}  # dictionary is made with tuples of top-left coordinates
    
    return square_start_pt, square_dict







def inner_square_points(square_dict,i,edge):
    """
    Generates the 4 square vertex coordinates, to later color the cube sketch
    These square vertex coordinates are shifted by 1 pixel to the inner side, based on the top-left square vertex (dict of top left
    vertex of the 54 facelets); The dict index defines the facelet number, and the edge is the square side lenght
    The returned array defines the 4 points coordinate, of the area within each of the 54 facelets."""
    
    x=square_dict[i][0]+1                                 # top left x coordinate is shifted by 1 pixel to the right
    y=square_dict[i][1]+1                                 # top left y coordinate is shifted by 1 pixel to the bottom
    d=edge-2                                              # square edge is reduced by 2 pixels 
    return np.array([(x,y),(x+d,y),(x+d,y+d),(x,y+d)])    # array with tupples of square coordinates, shifted by 1 pixel toward the inside







def plot_interpreted_colors(cube_status, detect_winner, cube_color_sequence, HSV_analysis, cube_status_string, \
                            collage, collage_w, collage_h, font, fontScale, lineType):
    """ Based on the detected cube status, a sketch of the cube is plot with bright colors on the pictures collage."""
    
    x_start=int(collage_w/1.9)      # top lef corner of the rectangle where all the cube's faces are plot
    y_start=int(collage_h/40)       # top lef corner of the rectangle where all the cube's faces are plot
    d = int(collage_w/40)           # edge lenght for each facelet reppresentation
    _, square_dict = cube_sketch_coordinates(x_start, y_start, d)     # dict with the top-left coordinates for each of the 54 facelets

    
    # some text info are placed close to the cube sketch
    cv2.putText(collage, 'Interpreted', (x_start+7*d, y_start+d), font, fontScale*1.2,(0,0,0),lineType)
    cv2.putText(collage, 'cube status:', (x_start+7*d, y_start+int(2.5*d)), font, fontScale*1.2,(0,0,0),lineType)
    if detect_winner == 'Error':     # case the cube status detection ended with an error
        # on the pictures collage a text is added indicating the Error
        cv2.putText(collage, str(detect_winner), (x_start+7*d, y_start+int(7.5*d)), font, fontScale*1.2,(0,0,0),lineType)
    else:  # case the cube status detection succeeded via BGR color distance or by HSV color analysis
        # on the pictures collage a text indicating which method was successfull (BGR color distance is done first, HSV only if BGR color distance fails)
        cv2.putText(collage, str(f'(via {detect_winner})'), (x_start+7*d, y_start+int(7.5*d)), font, fontScale*1.2,(0,0,0),lineType)
    
    cube_bright_colors = {'white':(255,255,255), 'red':(0,0,204), 'green':(0,132,0),
                          'yellow':(0,245,245), 'orange':(0,128,255), 'blue':(204,0,0)}   # bright colors assigned to the six faces colors
    std_color_sequence = list(cube_bright_colors.keys())                                  # URFDLB color order if white on top and red at right
    
    for i, color in enumerate(cube_status.values()):                     # iteration over the 54 facelets interpreted colors
        start_point=square_dict[i]                                       # top-left point coordinate for the facelet square
        cv2.rectangle(collage, tuple(start_point), (start_point[0]+d, start_point[1]+d), (0, 0, 0), 1) # square black frame
        if HSV_analysis:                                                 # case the detected 6 center facelets have 6 different colors
            col=cube_color_sequence[std_color_sequence.index(color)]     # from standard color sequence to the one detected, due to cube orientation at start
            B,G,R = cube_bright_colors[col]                              # BGR values of the bright colors for the corresponding detected color
            inner_points=inner_square_points(square_dict,i,d)            # array with the 4 square inner vertex coordinates
            cv2.fillPoly(collage, pts = [inner_points], color=(B,G,R))   # inner square is colored with bright color of the interpreted one
        else:                                                            # case the detected 6 center facelets do not have 6 different colors
            cv2.putText(collage, cube_status_string[i], (start_point[0]+int(0.2*d), int(start_point[1]+int(0.8*d))),\
                        font, fontScale*0.9,(0,0,0),lineType)            # facelets side LETTER is printed on the sketch

    





def robot_facelets_rotation(facelets):
    """ Rotates the facelets order, from robot's camera/cube orientation to the URFDLB point of view
    This has to do with the way the PiCamera is mounted on the robot, as well as how the faces are presented
    during the cube status reading.
    Argument is the dictionary of facelets (key:interpreted color) are retrieved by the robot
    Return is a dictionary of facelets (key:interpreted color) that follows the URFDLB order
    Ther return allows to have an interpreted cube status as per user point of view, standing in front of the robot."""

    # On sides 1, 3, 4, 5, PiCamera reads facelets at 180deg with reference to user point of view
    # These sides numbers refer to the face order during the cube status detection (U, B, D, F, R, L)
    if side in [1, 3, 4, 5]:
        facelets.reverse()   # reversing the order on the original facelet list solves the problem
    






def average_color(frame, x, y, edge):
    """ From: https://sighack.com/post/averaging-rgb-colors-the-right-way
     Averages the pixels within a square defined area on an image
     The average is calculated as the square root of the sum of the squares for the BGR colors
     region centered at (x, y) meant as the square center, with 2*side as quare side lenght in pixels.
     The function return a tuple with the averaged BGR colors
     Pixel by pixel iteration doesn't sound efficient, yet for the small aarea I couldn't notice a time increment."""
    
    # square edge, is used as (half) side of the square to calculate the averaged color
    
    blue=float(0)    # blue variable set as float
    green=float(0)   # green variable set as float
    red=float(0)     # red variable set as float
    
    #Iterate through pixels of a bounding box having 2*edge as square side length in pixels
    for i in range(2*edge):              # foor loop used to iterate the colums on the image square 
        j=i-edge                         # iterator j is "shifted" by half of the square of pixels to analyse
        for i in range(2*edge):          # for loops to iterate trhough the rows of the image square
            bgr=frame[y+j,x-edge+i]      # gbr of a single pixel
            b,g,r = bgr                  # bgr components
            b=int(b)                     # from uint8 to integer
            g=int(g)                     # from uint8 to integer
            r=int(r)                     # from uint8 to integer
            
            #Sum the squares of components
            blue += b*b                  # progressive sum of the square values for the blue component
            green += g*g                 # progressive sum of the square values for the green component
            red += r*r                   # progressive sum of the square values for the red component
    num=4*edge*edge                      # amount of pixels in the image square under analysis    
    
    # for debug purpose it is drawn the contour of the used area where the facelet's color is averaged 
    if debug and screen:                 # case debug and screed variables are set true on __main__
        tl=(x-edge, y-edge)              # top left coordinate 
        tr=(x+edge, y-edge)              # top right coordinate 
        br=(x+edge, y+edge)              # bottom left coordinate 
        bl=(x-edge, y+edge)              # bottom left coordinate 
        pts=np.array([tl, tr, br, bl])   # array of coordinates
        contour = [pts]                  # list is made with the array of coordinates
        cv2.drawContours(frame, contour, -1, (230, 230, 230), 2)  # a white polyline is drawn on the contour (2 px thickness)
    
    #Return the sqrt of the mean of squared B, G, and R sums 
    return (int(math.sqrt(blue/num)), int(math.sqrt(green/num)), int(math.sqrt(red/num)))







def read_color(frame, facelets, candidates, BGR_mean, H_mean, wait=20, index=0):
    """ Reads the average BGR color on the each facelet of the cube face just detected.
    Draw the contour used on each facelect (eventually the facelet number), to feedback on correct facelet reading/ordering
    Wait is the time (in ms) to keep each facelet visible while the remaining frame is forced black
    The function returns (or updates) global variables, like BGR_mean, hsv, hue, s, v, H_mean. """
    
    global edge
    
    if side==1:                             # case the read_color function is called on the first cube face
        area=0                              # are variable is set to zero
        for facelet in facelets:            # iteration over the 9 facelets detected on this cube face
            area+=facelet.get('area')       # total area of the contours of these 9 faceles
        
        # 3% of avg facelet contour area is used to calculate the edge length of a suare.
        # this square is later used to define a central (and small) square on each facelet, where to measure the average HSV
        edge = int(math.sqrt(area/270))
    
    for facelet in facelets:                                  # iteration over the 9 facelets just detected
        contour = facelet.get('contour')                      # contour of the facelet under analysis
        candidates.append(contour)                            # new contour is added to the candidates list
        mask = np.zeros(frame.shape[:2], dtype='uint8')       # mask of zeros is made for the frame shape dimension
        cv2.drawContours(mask, [contour], -1, 255, -1)        # mask is applied to vsualize one facelet at the time
        cm_point=facelet['cx'],facelet['cy']                  # contour center coordinate
        bgr_mean_sq = average_color(frame, facelet['cx'], facelet['cy'], edge)  # color is averaged with sqr sum of squares
        BGR_mean.append(bgr_mean_sq)                          # Initially used a simpler mean to average the facelet color
        b,g,r = bgr_mean_sq                                   # BGR (avg) components are retrieved
        BGR_mean_sq = np.array([[[b,g,r]]], dtype=np.uint8)   # BGR are positioned in cv2 array form
        hsv = cv2.cvtColor( BGR_mean_sq, cv2.COLOR_BGR2HSV)   # HSV color space equilavent values, for the average facelet color
        hue, s, v = cv2.split(hsv)                            # HSV components are retrieved
        H_mean.append((hue[0][0]))                            # the (avg) Hue value is stored on a list
        
        # a progressive facelet numer, 1 to 9, is placed over the facelets
        # the facelet order is the one from camera point of view, therefore before re-ordering according to user POV
        if debug:                                             # case debug variable is set true on __main__
            cv2.putText(frame, str(index+1), (int(facelet.get('cx'))-12, int(facelet.get('cy'))+6), font, fontScale,(0,0,0),lineType)
            
        index+=1






def face_image(frame, facelets, side, faces):
    """ Slice a frame rectangular portion to temporary store the cube face image.
    The cube face image is initialy cropped from the frame.
    1st vertex of Facelets 0, and 3rd vertedx of facelet 8, are used as reference for the cubre cropping from the frame
    The facelets are first re-ordered from top left
    The function returns a dictionary with the (cropped) images of the 6 cube faces
    This function enables the generation of a cube images collage to be plotted (and saved on Rpi), for decoration purpose
        
    A         B 
      0  1  2
      3  4  5 
      5  7  8  
    D         C   """
     
    if side in [1, 3, 4, 5]:                             # sides number according the robot scan order
        facelets.reverse()                               # facelets are ordered as per user POV

    Ax = int(facelets[0].get('cont_ordered')[0][0])      # x coordinate for the top-left vertex 1st facelet (0)
    Ay = int(facelets[0].get('cont_ordered')[0][1])      # y coordinate for the top-left vertex 1st facelet (0)
    Cx = int(facelets[8].get('cont_ordered')[2][0])      # x coordinate for the bottom-right vertex 9th facelet (8)
    Cy = int(facelets[8].get('cont_ordered')[2][1])      # y coordinate for the bottom-right vertex 9th facelet (8)
    diagonal = int(math.sqrt((Cy-Ay)**2+(Cx-Ax)**2))     # cube face diagonal length
    
    if frameless_cube == 'true':                         # case the cube detection status is set for cubes with frame around the facelets
        margin = int(marg_coef*diagonal)    #(AF 0.06)   # 6% of cube diagonal is used as crop margin (bandwidth outside the detected contours)
    else:                                                # case the cube detection status is set for frameless cubes
        margin = int(1.33*marg_coef*diagonal)            # larger value is used as as crop margin (bandwidth outside the detected contours)
    
    robot_facelets_rotation(facelets) # facelets are rotated to URFDLB related order
    
    h, w = frame.shape[:2]   # frame height and width
    
    if Ax >= margin:         # case the x vertex coordinate is bigger than the margin
        Ax = Ax-margin       # shifted coordinate
    else:                    # case the x vertex coordinate is not bigger than the margin
        Ax = 0               # zero is used as x coordinate for this vertex
    
    if Ay >= margin:         # case the y vertex coordinate is bigger than the margin
        Ay = Ay-margin       # shifted coordinate
    else:                    # case the y vertex coordinate is not bigger than the margin
        Ay = 0               # zero is used as y coordinate for this vertex
    
    if w-Cx >= margin:       # case the x vertex coordinate is more than margin apart from the frame width
        Cx = Cx+margin       # shifted coordinate
    else:                    # case the x vertex coordinate is less than margin apart from the frame width
        Cx = w               # frame width is used as x coordinate for this vertex
    
    if h-Cy >= margin:       # case the y vertex coordinate is more than margin apart from the frame height
        Cy = Cy+margin       # shifted coordinate
    else:                    # case the y vertex coordinate is less than margin apart from the frame height
        Cy = h               # frame height is used as x coordinate for this vertex

    
    # text over the cube face's images, for debug purpose
    if debug:                                # case debug variable is set true on __main__
        text_x = Ax + int(0.2*(Cx-Ax))       # X coordinate for the text starting location
        text_y = int((Ay+Cy)/1.8)            # Y coordinate for the text starting location
        fontscale_coef = (Cx-Ax)/150         # coefficient to adapt the text size to almost fit the cube
        cv2.putText(frame, str(f'Side {sides[side]}'), (text_x, text_y), font, fontScale*fontscale_coef, fontColor,lineType)
    
    faces[side] = frame[Ay:Cy, Ax:Cx]        # sliced image of "only" the cube face

    return faces






def robot_to_cube_side(side, cam_led_bright):
    """ Cube movements at robot, during the cube reading phase, to get access to all the faces.
    Cube is flipped 4 times to read the first 4 faces, then some spins and flippings are required to read the
    remaining 2 faces.
    Top cover remains open, so the picamera is at sufficient distance from the cube's side to be read.
    The led on top_cover is switched off before moving the servo, and on again afterward, to limit max current.
        
    From 26/11/2022, after scanning the status, the cube is NOT moved to the initial position anymore:
    Out of ca 1300 cube solving cycles (3 robots, random scrambled, removed dublicated status, etc),
    62% of the times the first move is one of the URF sides, where U leads with 23% of the total.
    After scanning the 6th cube face, the U face is perfectly on the bottom, so better to start from there."""
    
    if side==0:                                  # case side equals zero (used for preparing the next steps)
        if not robot_stop:                       # case there are not request to stop the robot
            servo.read()                         # top cover positioned to have the PiCamera in read position
            servo.cam_led_On(cam_led_bright)     # led on top_cover is switched on 
        
    elif side in (1,2,3):                        # first 4 sides (3 sides apart the one already in front of the camera)
        servo.cam_led_Off()                      # led on top_cover is switched off, for power management
        if not robot_stop:                       # case there are not request to stop the robot
            servo.flip()                         # are reached by simply flipping the cube
            servo.cam_led_On(cam_led_bright)     # led on top_cover is switched on 
    
    elif side == 4 :                             # at side 4 is needed to change approach to show side 5 to the camera
        servo.cam_led_Off()                      # led on top_cover is switched off, for power managements
        if not robot_stop:                       # case there are not request to stop the robot
            servo.spin_out('CW')                 # one spin (90 deg CW) is applied to the cube
        if not robot_stop:                       # case there are not request to stop the robot
            servo.flip()                         # one flip is applied to the cube to bring the side 5 on top
        if not robot_stop:                       # case there are not request to stop the robot
            servo.spin_home()                    # spin back home is applied to the cube to show the side 5 to the camera
        if not robot_stop:                       # case there are not request to stop the robot
            servo.read()                         # top cover positioned to have the PiCamera in read position
            servo.cam_led_On(cam_led_bright)     # led on top_cover is switched on
            
    elif side == 5 :                             # when side5 is in front of the camera
        servo.cam_led_Off()                      # led on top_cover is switched off, for power management
        for i in range(2):                       # at side 5 are needed two flips to show side 6 to the camera
            if not robot_stop:                   # case there are not request to stop the robot
                servo.flip()                     # cube flipping
        if not robot_stop:                       # case there are not request to stop the robot
            servo.cam_led_On(cam_led_bright)     # led on top_cover is switched on
  
    elif side == 6 :                             # case side equal 6 (it is when the cube has been fully scanned)
        servo.cam_led_Off()                      # led on top_cover is switched off






def robot_move_cube(robot_moves, total_robot_moves, solution_Text, start_time, scrambling=False):
    """This fuction calls the robot servo function to apply the solving movements to the cube; Arguments of this function are:
        - robot_moves, a string with the calculated movements for the robot based on the kociemba solution
        - total_robot_moves value, used to visualize on display a robot moves count-down
        - solution_Text, used to detect error cases on the Kociemba solution."""
    
    
    start_robot_time = time.time()    # this time is used as reference to measure (and visualize) how long the robot takes to solve the cube
#     remaining_moves = total_robot_moves  # remaining movements are visualized, to manage expectations while in front of the robot
    
    if not scrambling:                # case the robot is used to solve a cube
        print()                       # print an empty row
    robot_status, robot_time = servo.servo_solve_cube(robot_moves, scrambling, print_out=debug)   # robot solver is called
        
    if solution_Text == 'Error':      # if there is an error (tipicallya bad color reading, leading to wrong amount of facelets per color)                                      
        print('An error occured')                              # error feedback is print at terminal
        robot_solving_time = 0                                 # robot solving time is set to zero to underpin the error
        tot_robot_time = time.time()-start_time                # total robot time is calculated
        disp.show_on_display('DETECTION', 'ERROR', fs1=19)     # feedback is printed to the display
        solved = False                                         # solved variable is set false
        time.sleep(5)                                          # 5 secs delay is applied, to let user reading info on screen
    
    elif solution_Text != 'Error' and not robot_stop: # case there are not error on the cube solution and no request to stop the robot
        if not scrambling:                                     # case the robot is used to solve a cube
            solved = True                                      # solved variable is set true
            tot_robot_time, robot_solving_time = robot_time_to_solution(start_time, start_robot_time,\
                                                                        total_robot_moves)  # cube solved function is called
            disp.show_on_display('CUBE', 'SOLVED !', y1=22, fs1=36)     # feedback is printed to the display 
            if total_robot_moves != 0:                         # case the robot had to move the cube to solve it
                servo.fun(print_out=debug)                     # cube is rotated CW-CCW for few times, as victory FUN 'dance'
            disp.show_on_display(f'TOT.: {round(tot_robot_time,1)} s',\
                            f'SOLV.: {round(robot_solving_time,1)} s',\
                            x1=10, fs1=18, x2=10, fs2=18)      # time feedback is printed to the display
            if not screen:                                     # case a screen is not connected
                time.sleep(7)                                  # 7 secs delay is applied, to let user reading info on screen
            else:                                              # case a screen is connected
                time.sleep(1)                                  # 1 sec delay is applied, to let user reading info on screen
        
        elif scrambling:                                       # case the robot is used to scramble a cube
            disp.show_on_display('CUBE', 'SCRAMBLED', fs1=22, fs2=18)     # feedback is printed to the display
            solved = False                                     # solved variable is set false
            tot_robot_time = 0                                 # total robot time variable is set to zero
            robot_solving_time = 0                             # total robot solving time variable is set to zero
            time.sleep(2)                                      # 2 secs delay is applied, to let user reading info on screen
            
    else:                             # case there are not error on the cube solution, but there is a request to stop the robot
        robot_solving_time = 0                                 # robot solving time is set to zero
        tot_robot_time = time.time()-start_time                # total robot time is calculated
        disp.show_on_display('STOPPED', 'CYCLE')               # feedback is printed to the display
        solved = False                                         # solved variable is set false
        time.sleep(1)                                          # 1 secs delay is applied, to let user reading info on screen

    return solved, tot_robot_time, robot_solving_time






def text_font():
    """ Sets the (main) text parameters, used on CV2."""
    
    font = cv2.FONT_HERSHEY_SIMPLEX               # type of cv2 font used in this script
    fontScale = 0.8                               # font size used, when not (locally)  changed
    fontColor = (255,255,255)                     # fiont color, when not (locally) changed
    lineType = 2                                  # font thickness, when not (locally) changed
    return font, fontScale, fontColor, lineType






def robot_solve_cube(fixWindPos, screen, frame, faces, cube_status, cube_color_sequence, HSV_analysis, URFDLB_facelets_BGR_mean,
                        font, fontScale, lineType, show_time, timestamp, solution, solution_Text, color_detection_winner,
                        cube_status_string, BGR_mean, HSV_detected, start_time, camera_ready_time, cube_detect_time, cube_solution_time):
    
    """ Sequence of commands involving the robot, after the cube status detection
    Within this function the robot is called to solve the cube.
    This function calls many other functions, like a picture collage maker to store the detected images of the cube . """
    
    global robot_stop
    
    # dict and string with robot movements, and total movements
    _, robot_moves, total_robot_moves = rm.robot_required_moves(solution, solution_Text) 
#     print(f'\nRobot movements sequence: {robot_moves}')   # nice information to print at terminal, sometime useful to copy 
    
    if solution_Text != 'Error':                # case the solver has returned an error
        print(f'Total robot movements: {total_robot_moves}')  # nice information to print at terminal, sometime useful to copy

    if color_detection_winner == 'BGR':         # case the cube status has been positively detected by the BGR color distance method
        facelets_data=BGR_mean                  # data to be later logged in a text file         
    elif color_detection_winner == 'HSV':       # case the cube status has been positively detected by the HSV color analysis
        facelets_data=HSV_detected              # data to be later logged in a text file
    else:                                       # case the solver has returned an error
        facelets_data=BGR_mean, HSV_detected    # data to be later logged in a text file  
    
    if screen:                                  # case screen variable is set true on __main__
        try:
            cv2.destroyWindow('cube')           # cube windows is closed
        except:
            pass
     
    if not robot_stop:             # case there are no request to stop the robot
        # movements to the robot are finally applied
        solved, tot_robot_time, robot_solving_time = robot_move_cube(robot_moves, total_robot_moves, solution_Text, start_time)
        
        # some relevant info are logged into a text file
        log_data(timestamp, facelets_data, cube_status_string, solution, color_detection_winner, tot_robot_time, \
                 start_time, camera_ready_time, cube_detect_time, cube_solution_time, robot_solving_time)
        
        deco_info = (fixWindPos, screen, frame, faces, cube_status, cube_color_sequence,\
                     HSV_analysis, cube_status_string, URFDLB_facelets_BGR_mean, \
                     color_detection_winner, show_time, timestamp) # tuple of variables needed for the decoration function
        decoration(deco_info)  # cals the decoration function, that shows (or just saves, is screen=False) cube's faces pictures  
        
    else:                          # case there is a request to stop the robot
        tot_time_sec = 0           # robot solution time is forced to zero when the solving is interrupted by the stop button
   
    servo.servo_start_pos()        # servos are placed back to their start position







def robot_timeout_func():
    """ Robot reaction mode in case of timeout.
    Cube state reading, in case of high reflection or pollutted facelets, could get stuck therefore the need for a timeout;
    This function takes care of quitting the reading status."""
    
    servo.cam_led_Off()         # sets off the led at top_cover
    print(f'\nTimeout for cube status detection: Check if too much reflections, or polluted facelets')
    # feedback is printed to the terminal
    
    disp.show_on_display('READING', 'TIME-OUT', fs1=24)  # feedback is printed to the display
    time.sleep(5)               # time to let possible reading the display
    servo.servo_start_pos()     # top cover and cube lifter to start position

    if screen:                  # case screen variable is set true on __main__
        cv2.destroyAllWindows() # all the windows are closed
    
    timeout=True                # boolean variable (also global) used to manage the timeout case
    return timeout              # timeout is returned






def robot_time_to_solution(start_time, start_robot_time, total_robot_moves):
    """ Calculates the time the robot takes to read and solve the cube.
    Prints the total time, and the time used to manoeuvre the cube, to the solution
    Returns also a (global variable) boolean that the cube is solved, differently this function isn't called."""
    
    tot_robot_time = time.time() - start_time                   # total robot time from camera warmup until cube solved
    robot_solving_time = time.time() - start_robot_time         # robot time needed to manoeuvre the cube to solve it
    if total_robot_moves > 0 :                                  # case some manoeuvres to the cube are needed to solve it
        print(f'\nTotal time until cube solved: {round(tot_robot_time,1)} secs')  # feedback is printed to the terminal
    elif total_robot_moves == 0:                                # case no manoeuvres are needed to solve the cube
        print(f'\nCube was already solved')                     # feedback is printed to the terminal
        disp.show_on_display('ALREADY', 'SOLVED')               # feedback is printed to the display
        time.sleep(3)                                           # little delay to allows display reading

    return tot_robot_time, robot_solving_time







def log_data(timestamp, facelets_data, cube_status_string, solution, color_detection_winner, \
             tot_robot_time, start_time, camera_ready_time, cube_detect_time, cube_solution_time, robot_solving_time):
    
    """ Main cube info are logged in a text file
    This function is called obly on the robot (Rpi), to generate a database of info usefull for debug and fun
    BGR color distance is the first approach used to detect cube status, therefore the winner if it succedes.
    HSV color approach is used when the BGR approach fails; If the HSV succedes on cube status detection it become the winner.
    If the cube solver returns an error it means both the approaches have failed on detecting a coherent cube status."""
    
    folder = pathlib.Path().resolve()                   # active folder (should be home/pi/cube)  
    folder = os.path.join(folder,'CubesDataLog')        # folder to store the relevant cube data
    if not os.path.exists(folder):                      # if case the folder does not exist
        os.makedirs(folder)                             # folder is made if it doesn't exist
    
    fname = folder+'/Cubotino_solver_log.txt'           # folder+filename for the cube data
    if not os.path.exists(fname):                       # if case the file does not exist, file with headers is generated
        if debug:                                       # case debug variable is set true on __main__
            print(f'\ngenerated AF_cube_solver_log_Rpi.txt file with headers') # feedback is printed to the terminal
        
        a = 'Date'                                      # 1st column header
        b = 'FramelessCube'                             # 2nd column header
        c = 'ColorAnalysisWinner'                       # 3rd column header
        d = 'TotRobotTime(s)'                           # 4th column header
        e = 'CameraWarmUpTime(s)'                       # 5th column header
        f = 'FaceletsDetectionTime(s)'                  # 6th column header
        g = 'CubeSolutionTime(s)'                       # 7th column header
        h = 'RobotSolvingTime(s)'                       # 8th column header
        i = 'CubeStatus(BGR or HSV or BGR,HSV)'         # 9th column header
        k = 'CubeStatus'                                # 10th column header
        l = 'CubeSolution'                              # 11th column header
        s = a+'\t'+b+'\t'+c+'\t'+d+'\t'+e+'\t'+f+'\t'+g+'\t'+h+'\t'+i+'\t'+k+'\t'+l+'\n'  # tab separated string of the the headers
        
        os.umask(0) # The default umask is 0o22 which turns off write permission of group and others
        # 'a'means: file will be generated if it does not exist, and data will be appended at the end
        with open(os.open(fname, os.O_CREAT | os.O_WRONLY, 0o777), 'a') as f:    # text file is temporary opened
            f.write(s)               # data is appended


    # info to log
    a=str(timestamp)                                                   # date and time
    b=frameless_cube                                                   # frameless cube setting
    c=str(color_detection_winner)                                      # wich method delivered the coherent cube status
    d=str(round(tot_robot_time,1))                                     # total time from camera warmup to cube solved
    e=str(round(camera_ready_time-start_time,1))                       # time to get the camera gains stable
    f=str(round(cube_detect_time-camera_ready_time,1))                 # time to read the 6 cube faces
    g=str(round(cube_solution_time-cube_detect_time,1))                # time to get the cube solution from the solver
    h=str(round(robot_solving_time,1))                                 # time to manoeuvre the cube to solve it
    i=str(facelets_data)                                               # according to which methos delivered the solution (BGR, HSV, both)
    k=str(cube_status_string)                                          # string with the detected cbe status
    l=str(solution)                                                    # solution returned by Kociemba solver
    s = a+'\t'+b+'\t'+c+'\t'+d+'\t'+e+'\t'+f+'\t'+g+'\t'+h+'\t'+i+'\t'+k+'\t'+l+'\n'      # tab separated string with info to log
    
    # 'a'means: file will be generated if it does not exist, and data will be appended at the end
    with open(fname,'a') as f:   # text file is temporary opened
        f.write(s)               # data is appended







def camera_opened_check():
    """ Trivial check if camera is opened, by interrogating it on one of the settings; Funtion returns a boolean."""

    try:
        binning = camera.sensor_mode    # PiCamera sensor_mode is inquired to the camera
        if binning >= 0:                # case the returned parameter (binning) is bigger than zero
            return True                 # camera is opened, and true is returned
    except:                             # except is raised if camera isn't opened or does not properly reply on the inquiry
        return False                    # camera is evaluated as closed, and false is returned







def close_camera():
    """ Closes the camera object; It's important to close the camera, if the cube detection is performed more than once.
    On PiCamera it's importan to close it, at the end of a cube solving cycle to drop the AWB and Exposure setting used before."""
    
    try:
        camera.close()                  # necessary to close the camera to release the fix settings, like analog/digital gains
        if debug:                       # case debug variable is set true on __main__
            print(f'\nClosed camera')   # feedback is printed to the terminal

    except:
        pass







def robot_set_servo():
    """ The robot uses a couple of servos; This functions positions the servos to the start position."""
    
    return servo.init_servo()   # servos are initialized, and set to their starting positions
    







def stop_cycle(touch_btn):
    """ Function called as an interrupt in case the "start/stop button" is pressed.
    Time delay is used to prevent unwanted interruptions: Button has to be pressed at least for 0.5 seconds
    The function change (global) variables used on robot movements, to prevent further movements to happen
    The function calls the quitting function, that closes the script."""
    
    global robot_stop
    
    time.sleep(0.5)  # delay between function being called, and new GPIO check, to filter unintentionally touches
    if GPIO.input(touch_btn) and not robot_idle:     # case touch button is 'pressed' while robot not idling
        
        if not robot_stop:                           # in case the robot was working and not yet stopped
            servo.cam_led_Off()                      # sets off the led at top_cover
            disp.set_backlight(1)                    # display backlight is turned on, in case it wasn't
            disp.show_on_display('STOPPED', 'CYCLE') # feedback is printed to the display
            robot_stop = True                        # global flag to immediatly interrup the robot movements is set
            servo.stopping_servos(print_out=debug)   # function to stop the servos    
            time.sleep(1)
            disp.show_on_display('STOPPED', 'CYCLE') # feedback is printed to the display a second time
            quit_func(quit_script=False)             # quit function is called, without forcing the script quitting

        





def robot_set_GPIO():
    """ Raspberry Pi requires some settings at the GPIO (General Purpose imput Output)
    This function sets the GPIO way of working
    This function also sets an interrupt for the start/stop button."""
    
    global GPIO, touch_btn
 
    GPIO.setwarnings(False)                                         # GPIO warning set to False to reduce effort on handling them
    GPIO.setmode(GPIO.BCM)                                          # GPIO modulesetting
    touch_btn = 26                                                  # GPIO pin used for the touch button (start/stop button)
    GPIO.setup(touch_btn, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)      # set the touch_button_pin as an input
    try:
        # interrupt usage (of the same input pin), to quicly stop the robot movements
        GPIO.add_event_detect(touch_btn, GPIO.RISING, callback=stop_cycle, bouncetime=20)
    except:
        pass


###### addition for faire demo setup #####
# it uses two inputs in logic AND to start the robot. These buttons don't do enything else. No pressing time filters
    global touch_btn1_faire, touch_btn2_faire
    touch_btn1_faire = 23                                              # GPIO pin used for the touch button1 at faire (only start function)
    GPIO.setup(touch_btn1_faire, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # set the touch_button_pin as an input
    touch_btn2_faire = 24                                              # GPIO pin used for the touch button2 at faire (only start function)
    GPIO.setup(touch_btn2_faire, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # set the touch_button_pin as an input
##########################################
    








def cpu_temp(side, delay=10):
    """ Funtion to read/print CPU temperature at Raspberry pi.
    This gives an idea about the temperature into the robot case."""
    
    try:
        tFile = open('/sys/class/thermal/thermal_zone0/temp')   # file with the cpu temp, in mDegCelsius (text format)
        cpu_temp = round(float(tFile.read()) /1000, 1)          # tempertaure is converted to (float) degCelsius

    except:           # case of errors
        tFile.close() # file is closed
    
    if side!=0:                                                                     # case the cube side is not zero 
        print(f'\nCPU temperature: {cpu_temp} degrees C')                           # feedback is printed to the terminal
        disp.show_on_display('CPU TEMP.', str(str(cpu_temp)+' degC'), fs1=20, fs2=20)    # feedback is printed to the display
        import time
        time.sleep(delay) # sleep time in arg is applied








def time_system_synchr():
    """ Checks the time system status; In case of internet connection, waits for synchronization before proceeding.
        In case of internet connection, a max 20 synchronization attempts (10 seconds) are done.
        This choice to prevent time mismatch when the synchronization happens during a cube solving process.
        Raspberry pi doesn't have an RTC, and time module updates once an internet connection is made.
        When the time module is initially synchronized, there will also be later adjustments (I believe every 5 minutes);
        these later adjustments aren't of a problem for the time calculation in this script."""
    
    import socket
    
    try:
        res = socket.getaddrinfo('google.com',80)       # trivial check if internet is available
        print('internet is connected')                  # feedback is printed to the terminal
        internet = True                                 # internet variable is set true
    except:                                             # exception is used as no internet availability
        print('no internet connection')                 # feedback is printed to the terminal
        internet = False                                # internet variable is set false
    
    if internet:                                        # case internet is available
        from subprocess import Popen, PIPE              # classes importing        
        once = True                                     # variable once is set true, to print a feedback only once
        i = 0                                           # iterator
        while True:                                     # infinite loop              
            try:
                if i == 20:                             # case the iteration has been done 20 times (10 seconds)
                    break                               # while loop is interrupted
                
                # inquiry to timedatectl status 
                ps = Popen("timedatectl status | grep 'System clock synchronized'  | grep -Eo '(yes|no)'", shell=True, stdout=PIPE)
                output = ps.stdout.read()                        # process output
                ps.stdout.close()                                # closing the pipe
                ps.wait()                                        # waits until the ps child completes
                
                if b'yes' in output:                             # case the timedatectl status returns true
                    date_time = dt.datetime.now().strftime('%d/%m/%Y %H:%M:%S')   # updated date and time assigned to date_time variable
                    print('time system is synchronized: ', str(date_time))        # feedback is printed to the terminal
                    disp.show_on_display('TIME SYSTEM','UPDATED', fs1=16)   # feedback is printed to the display
                    time.sleep(1.5)
                    disp.show_on_display(str(date_time[11:]), '', fs1=26)              # feedback is printed to the display
                    break                                        # while loop is interrupted
                else:                                            # case the timedatectl status returns false
                    if once:                                     # case the variable once is true
                        print('waiting for time system update')  # feedback is printed to the terminal
                        once = False                             # variable once is set false, to print a feedback only once
                    time.sleep(0.5)                              # sleep time before inquiry to timedatectl status again
                    i+=1                                         # iterator is increased
            except:                                              # case there is an exception
                break                                            # while loop is interrupted
                
    else:                                                        # case the is not an internet connection
        print('time system not synchronized yet')                # feedback is printed to the terminal
        disp.show_on_display('TIME SYSTEM','NOT UPDATED', fs1=16, fs2=15)   # feedback is printed to the display
        time.sleep(1.5)






def clear_terminal():
    """ Clears the terminal and positions the cursors on top left; Still possible to scroll up in case of need"""

    print("\x1b[H\x1b[2J")  # escape sequence







def start_scrambling(scramb_cycle):
    """ 1) starts a scrambling cube cycle
        2) prints on terminal before and after the scrambling
        3) manage the scrambling end, when interrupted.""" 

    global quitting, robot_stop, robot_idle
    
    print('\n\n\n\n\n\n')       # prints some empty lines, on IDE terminal
    clear_terminal()            # cleares the terminal
    print('#############################################################################')
    print(f'########################    SCRAMBLING CYCLE  {scramb_cycle}    ##########################')
    print(f'#############################################################################\n')

    scrambling_cube()           # call to the function for random cube generation
    
    if not robot_stop:          # case the robot has not been stopped
        print(f'\n######################    END  SCRAMBLING CYCLE  {scramb_cycle}   ########################')
    
    elif robot_stop:            # case the robot has been stopped
        stop_or_quit()          # check if the intention is to stop the cycle or quit (shut Rpi off)
        quitting = False        # flag used during the quitting phase
        robot_stop = False      # flag used to stop or allow robot movements
        robot_idle = True       # robot is idling
        print(f'\n####################    STOPPED  SCRAMBLING CYCLE  {scramb_cycle}   ######################')







def start_solving(solv_cycle):
    """ 1) starts a solving cube cycle
        2) prints on terminal before and after the scrambling
        3) manage the solving end, when interrupted.""" 
      
    print('\n\n\n\n\n\n')       # prints some empty lines, on IDE terminal
    clear_terminal()            # cleares the terminal
    print('#############################################################################')
    print(f'#########################    SOLVING CYCLE  {solv_cycle}    ############################')
    print('#############################################################################') 

    cubeAF()                    # cube reading/solving function is called

    # handling the situation(s) after a robot cycle has been done or interrupted
    if robot_stop:              # case the robot has been stopped 
        stop_or_quit()          # check if the intention is to stop the cycle or quit (shut Rpi off)
        print(f'\n#####################    STOPPED  SOLVING CYCLE  {solv_cycle}   ########################')       

    elif timeout:               # case the cube status detection has reached the timeout
        print(f'\n####################    DETECTION TIMEOUT CYCLE  {solv_cycle}    #######################')        

    elif not (robot_stop or timeout): # case the robot has not been stopped, or it has reach the timeout
        disp.set_backlight(1)   # display backlight is turned on, in case it wasn't
        cpu_temp(side, delay=3) # cpu temperature is verified, printed at terminal and show at display for delay time
        print(f'\n#######################    END  SOLVING CYCLE  {solv_cycle}   ##########################')
    
    reset_camera = True         # reset_camera is set true, to reset settings of previous cycle
    return reset_camera







def start_automated_cycle(cycle, total, cycle_pause):
    """ 1) starts a scrambling cycles, followed by a solving cube cycle
        2) prints on terminal before and after the each of these cycles
        3) applies the set pause time in between automated cycles."""
    
    global robot_idle
    
    disp.set_backlight(1)       # display backlight is turned on, in case it wasn't
    disp.show_on_display('SCRAMBLING', f'{cycle} / {total} ', fs1=16, fs2=26)  #feedbak is print to to the display
    time.sleep(5)               # time delay to let possible readig the screen
    start_scrambling(cycle)     # start_scrambling function is called
    
    disp.set_backlight(1)       # display backlight is turned on, in case it wasn't
    disp.show_on_display('SOLVING', f'{cycle} / {total}', fs1=22, fs2=26)  #feedbak is print to to the display
    time.sleep(5)               # time delay to let possible readig the screen
    start_solving(cycle)        # start_solving function is called
    
    if cycle < total:                     # case there is at least another cycle        
        robot_idle = False                # robot idle set off to allows cycle stopping while pause
        disp.set_backlight(1)             # display backlight is turned on, in case it wasn't
        start = time.time()               # time reference
        screen1=True                      
        while time.time()-start < cycle_pause: # case the elapsed time is smaller than cycle_pause time
            time_left = int(cycle_pause-(time.time()-start))  # time_left is calculated and converted to integer
            
            if robot_stop:                # case of request to stop the robot
                stop_or_quit()            # stop or quit function is called
                break                     # while loop is interrupted
            
            else:                         # case no request to stop the robot
                if time_left % 2 == 0:    # case the time_left is even
                    if screen1:           # case boolean screen1 is true
                        screen1 = False   # boolean screen1 is set false
                    else:                 # case boolean screen1 is false
                        screen1 = True    # boolean screen1 is set true
                if screen1:               # case booleaa screen1 is true
                    disp.show_on_display('FOR CYCLE', f'{cycle+1} / {total}', fs1=21, fs2=24)  #feedbak is print to to the display
                else:                     # case booleaa screen1 is flase
                    disp.show_on_display('WAIT TIME', f'{time_left} s', fs1=21, fs2=24)  #feedbak is print to to the display
                
                if time_left <= 1:        # case the time to wait is almost over
                    break                 # the while loop is interrupted     
                else:                     # case there still is time to wait
                    time.sleep(1.9)       # system can sleep for almost 2 seconds





def robot_reset():
    """reset the servo to start position, and reset the display."""
    
    global robot_stop
    
    disp.set_backlight(1)                  # display backlight is turned on, in case it wasn't
    disp.show_on_display('ROBOT', 'RESTARTING', fs1=30, fs2=18) # feedback is printed to display
    time.sleep(2)                          # delay time to let user reading the display
    servo.stop_release(print_out=debug)    # stop_release at servo script, to enable the servos movements
    servo.servo_start_pos()                # servos are rotated to the start position
    robot_stop = False                     # flag used to stop or allow robot movements
    if not GPIO.input(touch_btn):          # case the button has been released
        disp.clean_display()               # cleans the display
        disp.__init__() # display is re-initilized (not elegant, yet it removes random issues at robot stop)
    






def stop_or_quit():
    """Distinguishes the request to just stop a cycle (scrambling or solving) or to SHUT the Rpi OFF.
        The input is the touch button and time.
        When the button is pressed longer than warning_time, a warning message is displayed.
        When the button is not released within the quit_time, the Rpi SHUT-OFF."""
    
    print('stop request')                    # feedback is printed to the terminal
    ref_time=time.time()                     # reference time used to measure how long the button has been kept pressed
#     warn_time = 1.5         #(AF 1.5)        # delay used as threshold to print a warning on display
#     quit_time = 4.5         #(AF 4.5)        # delay used as threshold to quit the script
    warning = False                          # warning is set false, to warn user to keep or release the button
    if GPIO.input(touch_btn):                # case touch_btn is still 'pressed' once the cube_AF function returns           
        while GPIO.input(touch_btn):                      # while touch_btn is 'pressed'
            if not warning:                               # case warning is false
                if time.time() - ref_time >= warn_time:   # case time elapsed is >= warn time reference
                    warning = True                        # warning is set true

            while warning:                                # case warning is true
                disp.set_backlight(1)                     # display backlight is turned on, in case it wasn't
                disp.show_on_display('SURE TO', 'QUIT ?') # feedback is printed to display
                if not GPIO.input(touch_btn):             # case the touch_btn is 'released'
                    disp.clean_display()                  # cleans the display
                    disp.show_on_display('NOT', 'QUITTING', fs1=32, fs2=22) # feedback is printed to display    
                    break                                 # while loop is interrupted
                
                if time.time() - ref_time >= quit_time:   # case time elapsed is >= quit time reference
                    print('quitting request')             # feedback is printed to display
                    for i in range(5):                    
                        disp.show_on_display('SHUTTING', 'OFF', fs1=20, fs2=28) # feedback is printed to display
                    servo.stop_release(print_out=debug)   # stop_release at servo script, to enable the servos movements
                    servo.init_servo(print_out=debug)     # servos are moved to their starting positions
                    quit_func(quit_script=True)           # qutting function is called, with script clossure
    
    time.sleep(5)       # time to mask the touch button pressing time (interrupt), eventually used to quit the script
    robot_reset()       # call the robot reset function






def quit_func(quit_script):   
    """ Quitting function, that properly closes stuff: Camera is closed, OpenCV windows are closed,
        Servos are positioned to their start position, Led at top_cover is switched off, set GPIO used for PWM to low level."""

    global quitting
    
    if not quit_script:            # case the quit_script variable is false (tipically every time this function is called)
        try:
            disp.set_backlight(1)                   # display backlight is turned on, in case it wasn't
            disp.show_on_display('STOPPED', 'CYCLE')     # feedback is printed to the display
            if not quitting:                        # case quitting variable is false, meaning all the times entering the quit_func
                quitting = True                     # (global) quitting variable is set true
        except:
            pass
        
        try:
            cv2.destroyAllWindows()     # closes al the graphical windows
        except:
            print("raised exception while cv2.destroyAllWindows at script quitting") # feedback is printed to the terminal
            pass
       
        try:
            servo.cam_led_Off()         # sets off the led at top_cover 
        except:
            print("raised exception while servo.cam_led_Off at script quitting")    # feedback is printed to the terminal
            pass

    
    
    ##################### closing the python script, and via bash shutting off the raspberry OS ########
    elif quit_script:                             # case the quit_script is set true (long button pressing)
        print('script quitting request')          # feedback is printed to the terminal
        
        try:                                      # tentative
            disp.set_backlight(1)                 # display backlight is turned on, in case it wasn't
            disp.show_on_display('SHUTTING', 'OFF', fs1=20, fs2=28) # feedback is printed to the display
            time.sleep(1.5)                       # wait time to let the message visible on the display
            
            countdown = 3                         # count-down variable
            if cover_self_close:                  # case the cover_self_close is set true at the settings file
                disp.show_on_display('CLOSING', 'COVER',fs1=24, fs2=26) # feedback is printed to the display
                time.sleep(1.5)                   # wait time to let the message visible on the display
                for i in range(countdown,-1, -1): # iteration down the countdown variable
                    dots = ''                     # dot string variable is set empty
                    for k in range(min(i,3)):     # iteration over the residual cont-down, with max of three
                        dots = dots + '.'         # dot string variable adds a dot character          
                    row2_text = str(i) + dots     # string variable to be printed on the second disply row
                    disp.show_on_display('CLOSING IN', row2_text, x1=20, y1=20, x2=20, y2=50, fs1=18, fs2=60)# feedback is printed to the display
                    if i > 0:                     # case the cont-down is above 0
                        time.sleep(1)             # wait time to let the message visible on the display
                servo.close_cover()               # top cover is closed
                time.sleep(0.5)                   # time to allow the servo reaching the close position
            
            disp.set_backlight(0)                 # display backlight is turned off
        except:
            pass
        
        try:
            servo.quit_func()       # sets to low the GPIO pins used as output
        except:
            print("raised exception while servo.quit_func at script quitting")   # feedback is printed to the terminal
            pass
         
        try:
            cv2.destroyAllWindows()     # closes al the graphical windows
        except:
            print("raised exception while cv2.destroyAllWindows at script quitting")   # feedback is printed to the terminal
            pass
       
        try:
            servo.cam_led_Off()         # sets off the led at top_cover 
        except:
            print("raised exception while servo.cam_led_Off at script quitting")   # feedback is printed to the terminal
            pass
        
        try:
            close_camera()          # closes the camnera object (should be the latest command, as per close camera)
        except:
            print('Issues at camera closing while quitting')    # feedback is printed to the terminal
            pass
        
        try:
            disp.clean_display()         # cleans the display
        except:
            print("raised exception while clean_display at script quitting")   # feedback is printed to the terminal
            pass
        
        try:
            disp.set_backlight(0)   # display backlight is turned on for the first time
        except:
            pass
        
        exit()
    
    else:
        print('something wrong at def quit_func')   # feedback is printed to the terminal








def start_up(first_cycle=False):
    """ Start up function, that aims to run (once) all the initial settings needed."""
    
    # global variables
    global camera, width, height, w, h, rawCapture, camera_set_gains       # camera and frame related variables
    global fps_i, fps_dict, show_time, cam_led_bright                      # camera and frame related variables
    global sides, side, faces, prev_side, BGR_mean, H_mean, URFDLB_facelets_BGR_mean      # cube status detection related variables                                 # facelet square side (in pixels) to draw the cube sketch
    global timeout, detect_timeout, robot_stop                             # robot related variables
    global font, fontScale, fontColor, lineType                            # cv2 text related variables

    
    
    # series of variables settings, to re-set at each cycle
    fps_dict={}                      # dict used to calculate the fps
    fps_i=0                          # index used to calculate the fps
    prev_side=0                      # set the initial previous side, when the fps is calculated
    BGR_mean=[]                      # empty list to be filled with with 54 facelets BGR colors while reading cube status
    H_mean=[]                        # empty list to be filled with with 54 facelets HUE values, while reading cube status
    URFDLB_facelets_BGR_mean=[]      # empty list to be filled with with 54 facelets colors, ordered according URFDLB order
    faces={}                         # dictionary that store the image of each face
    side=0                           # set the initial cube side (cube sides are 1 to 6, while zero is used as starting for other setting)
    cpu_temp(side)                   # cpu temp is checked at start-up and cube solving end
    robot_stop = False               # flag to stop the robot movements
    timeout = False                  # timeout flag is initialli set on False
    
    # series actions, or variables setting, to be done only at the first cycle
    if first_cycle:
        time_system_synchr()  # checks the time system status (if internet connected, it waits till synchronization)
#         cam_led_bright = 0.1           #(AF 0.1)           # set the brighteness on the led at top_cover (admitted 0 to 0.3)
#         detect_timeout = 40             #(AF 40)            # timeout for the cube status detection (in secs)
#         show_time = 7                      #(AF 7)             # showing time of the unfolded cube images (cube initial status)
        
        if cv_wow:                                             # case the cv image analysis plot is set true
            detect_timeout = 2 * detect_timeout                # cube status detection timeout is doubled
        sides={0:'Empty',1:'U',2:'B',3:'D',4:'F',5:'R',6:'L'}  # cube side order used by the robot while detecting facelets colors
        font, fontScale, fontColor, lineType = text_font()     # setting text font paramenters
        camera, rawCapture, width, height = webcam()           # camera relevant info are returned after cropping, resizing, etc
        robot_set_GPIO()                                       # GPIO settings used on the Raspberry pi
        robot_init_status = robot_set_servo()                  # settings for the servos
        if not robot_init_status:                              # case the servo init function returns False
            disp.set_backlight(1)                              # display backlight is turned on, in case it wasn't
            disp.show_on_display('SERVOS', 'ERROR', fs1=26, fs2=26) # feedback is printed to display
            time.sleep(5)
            disp.show_on_display('SHUTTING', 'OFF', fs1=20, fs2=28) # feedback is printed to display
            time.sleep(5)
            quit_func(quit_script=True)                        # qutting function is called, with script clossure
    








def cubeAF():
    """ This function is substantially the main function, covering all the different phases after the initial settings:
        Camera setting for 1st side and remaining
        Keeps interrogating the camera
        Cube status detection
        Cube solver call
        Cube solving at robot."""
    
    # global variables
    global camera, rawCapture, width, height, h, w, cam_led_bright, fixWindPos, screen    # camera and frame related variables          
    global sides, side, prev_side, faces, BGR_mean, H_mean, URFDLB_facelets_BGR_mean      # cube status detection related variables
    global font, fontScale, fontColor, lineType                                           # cv2 text related variables
    global servo, robot_stop, robot_idle, timeout, detect_timeout                         # robot related variables


    robot_idle = False                              # robot is not anymore idling
    side = 0                                        # side zero is used for some initialization processes
    
    if not camera_opened_check():                   # checks if camera is responsive
        print('\nCannot open camera')               # feedback is printed to the terminal
        disp.show_on_display('CAMERA', 'ERROR')     # feedback is printed to the display
        time.sleep(10)                              # delay to allows display to be read
        quit_func(quit_script=True)                 # script is closed, in case of irresponsive camera
    
    start_time = time.time()                        # initial time is stored before picamera warmup and setting
    faces.clear()                                   # empties the dict of images (6 sides) recorded during previous solving cycle
    facelets = []                                   # empties the list of contours having cube's square characteristics
    robot_to_cube_side(side, cam_led_bright)        # robot set with camera on read position
    servo.cam_led_On(cam_led_bright)                # led on top_cover is switched on before the PiCamera warmup phase     
    PiCamera_param = robot_camera_warmup(camera, start_time)    # calls the warmup function for PiCamera
    
    if not robot_stop:                              # case there are no requests to stop the robot
        robot_consistent_camera_images(camera, PiCamera_param, start_time)  # sets PiCamera to capture consistent images
        timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S') # date_time variable is assigned, for file name and log purpose
        camera_ready_time=time.time()               # time stored after picamera warmup and settings for consistent pictures
        side = 1                                    # side is changed to 1, as the cube faces are numbered from 1 to 6


    while not robot_stop:                           # substantially the main loop, it can be interrupted by quit_func() 
        if robot_stop:                              # case the robot has been stopped
            break                                   # while loop is interrupted
        
        # feedback is printed to the display
        disp.show_on_display('READING FACE', str(sides[side]), x1=15, y1=15, x2=50, y2=35, fs1=16, fs2=80)
        
        frame, w, h = read_camera()                          # video stream and frame dimensions
        
        if not robot_stop:                                   # case there are no requests to stop the robot
            (contours, hierarchy)=read_facelets(frame, w, h) # reads cube's facelets and returns the contours
            candidates = []                                  # empties the list of potential contours
        
        if not robot_stop and hierarchy is not None:         # analyze the contours in case these are previously retrieved
            hierarchy = hierarchy[0]                         # only top level contours (no childs)
            facelets = []                                    # empties the list of contours having cube's square characteristics
            
            if timeout or robot_stop:                        # in case of reached timeout or stop_button pressed
                quit_func(quit_script=False)                 # quit function is called, withou forcing the script quitting
                break                                        # while loop is interrupted
            
            for component in zip(contours, hierarchy):                # each contour is analyzed   
                contour, hierarchy, corners = get_approx_contours(component)  # contours are approximated
    
                if  time.time() - camera_ready_time > detect_timeout:  # timeout is calculated for the robot during cube status reading
                    timeout = robot_timeout_func()                     # in case the timeout is reached
                    break                                              # for loop is interrupted
                
                if robot_stop:                                         # case the robot has been stopped
                    break                                              # for loop is interrupted
                
                if not robot_stop and screen:                          # case screen variable is set true on __main__
                    if fixWindPos:                                     # case the fixWindPos variable is chosen
                        cv2.namedWindow('cube')                        # create the cube window
                        cv2.moveWindow('cube', 0,0)                    # move the cube window to (0,0)
                    cv2.imshow('cube', frame)                          # shows the frame 
                    cv2.waitKey(1)      # refresh time is minimized to 1ms, refresh time mostly depending to all other functions
                
                if corners==4:                                         # contours with 4 corners are of interest
                    facelets = get_facelets(facelets,frame, contour, hierarchy) # returns a dict with cube compatible contours

                if len(facelets)==9:                                   # case there are 9 contours having facelets compatible characteristics
                    facelets = order_9points(facelets, new_center=[])  # contours are ordered from top left
                    d_to_exclude = distance_deviation(facelets)        # facelets to remove due inter-distance not as regular 3x3 array
                    if len(d_to_exclude)>=1:                           # check if any contour is too far to be part of the cube
                        d_to_exclude.sort(reverse=True)                # reverse the contours order
                        for i in d_to_exclude:                         # remove the contours too faar far to be part of the cube
                            facelets.pop(i)                            # facelet is removed
                
                if len(facelets)==9:                                               # case having 9 contours compatible to a cube face
                    robot_facelets_rotation(facelets)                              # order facelets as per viewer POW (due to cube/camera rotations on robot)
                    read_color(frame, facelets, candidates, BGR_mean, H_mean)      # each facelet is read for color, decoration for the viewer is made
                    URFDLB_facelets_BGR_mean = URFDLB_facelets_order(BGR_mean)     # facelets are ordered as per URFDLB order             
                    faces = face_image(frame, facelets, side, faces)               # image of the cube side is taken for later reference
                    if not robot_stop and screen:                # case screen variable is set true on __main__
                        if cv_wow:                               # case the cv image analysis plot is set true                              
                            show_cv_wow(frame, time=2000)        # call the function that shows the cv_wow image
                        else:                                    # case the cv image analysis plot is set false
                            if fixWindPos:                       # case the fixWindPos variable is chosen
                                cv2.namedWindow('cube')          # create the cube window
                                cv2.moveWindow('cube', 0,0)      # move the window to (0,0)
                            cv2.imshow('cube', frame)            # shows the frame 
                            cv2.waitKey(1)                       # refresh time is minimized (1ms), yet real time is much higher
                    
                    disp.clean_display()                         # cleans the display
                    robot_to_cube_side(side, cam_led_bright)     # cube is rotated/flipped to the next face

                    if side < 6:                                 # actions when a face has been completely detected, and there still are other to come
                        if not robot_stop and screen:            # case screen variable is set true on __main__
                            if fixWindPos:                       # case the fixWindPos variable is chosen
                                cv2.namedWindow('cube')          # create the cube window
                                cv2.moveWindow('cube', 0,0)      # move the window to (0,0)
                            cv2.imshow('cube', frame)            # frame is showed to viewer
                            cv2.waitKey(1)                       # delay for viewer to realize the face is aquired
                        side +=1                                 # cube side index is incremented
                        break                                    # with this break the process re-start from contour detection at the next cube face

                    if side == 6:                                # last cube's face is acquired
                        disp.clean_display()                     # cleans the display
                        servo.cam_led_Off()                      # led at top_cover is set off         
                        cube_detect_time = time.time()           # time stored after detecteing all the cube facelets
                        if screen:                               # case screen variable is set true on __main__
                            try:
                                cv2.destroyAllWindows()          # cube window and eventual other open windows are close
                            except:
                                pass
                        
                        # cube string status with colors detected 
                        cube_status, HSV_detected, cube_color_seq, HSV_analysis = cube_colors_interpr(URFDLB_facelets_BGR_mean)
                        cube_status_string = cube_string(cube_status)                 # cube string for the solver
                        solution, solution_Text = cube_solution(cube_status_string)   # Kociemba solver is called to have the solution string
                        color_detection_winner='BGR'                                  # variable used to log which method gave the solution
                        cube_solution_time=time.time()                                # time stored after getting the cube solution
                        print(f'\nCube status (via BGR color distance): {cube_status_string}')   # feedback is printed to the terminal
                        print(f'Camera warm-up, camera setting, cube status (BGR), and solution, in: {round(time.time()-start_time,1)} secs')
                        
################### DEBUG #############             
#                         solution_Text = 'Error' # uncoment this row to force HSV color analysis method (this assignes 'Error' to solution_Text variable)
#######################################             

                        if solution_Text == 'Error':               # if colors interpretation on BGR color distance fail an attempt is made on HSV
                            print(f'Solver return: {solution}\n')  # feedback is printed to the terminal
                            a, b, c = cube_colors_interpr_HSV(URFDLB_facelets_BGR_mean,HSV_detected) # cube string status with colors detected
                            cube_status, cube_status_HSV, cube_color_seq = a, b, c        # cube string status with colors detected to variables with proper name
                            cube_status_string = cube_string(cube_status)                 # cube string for the solver
                            solution, solution_Text = cube_solution(cube_status_string)   # Kociemba solver is called to have the solution string
                            color_detection_winner='HSV'                                  # variable used to log which method give the solution
                            cube_solution_time=time.time()                                # time stored after getting the cube solution
                            print('Camera warm-up, camera setting, cube status (HSV), and solution, in:', round(time.time()-start_time,1))
                            if solution_Text == 'Error':                   # in case color color detection fail also with HSV approach
                                color_detection_winner='Error'             # the winner approach goes to error, for log purpose
                            else: 
                                print(f'\nCube status (via HSV color distance): {cube_status_string}')  # nice information to print at terminal, sometime useful to copy
                                print(f'\nCube solution: {solution_Text}') # nice information to print at terminal, sometime useful to copy 

      
                        elif solution_Text != '0 moves  ':                 # case of interest, the cube isn't already solved
                            print(f'\nCube solution: {solution_Text}')     # nice information to print at terminal, sometime useful to copy 
                        

                        # function related to cube solving via the robot
                        robot_solve_cube(fixWindPos, screen, frame, faces, cube_status, cube_color_seq, HSV_analysis, 
                                            URFDLB_facelets_BGR_mean, font, fontScale, lineType, show_time, timestamp,
                                            solution, solution_Text, color_detection_winner, cube_status_string, BGR_mean,
                                            HSV_detected, start_time, camera_ready_time, cube_detect_time, cube_solution_time) 
                        
                        return              # closes the cube reading/solver function in case it reaches the end
                
                
                if not robot_stop and screen:        # case screen variable is set true on __main__
                    if fixWindPos:                   # case the fixWindPos variable is chosen
                        cv2.namedWindow('cube')      # create the cube window
                        cv2.moveWindow('cube', 0,0)  # move the window to (0,0) corrdinate
                    cv2.imshow('cube', frame)        # shows the frame 
                    cv2.waitKey(1)                   # refresh time is minimized to 1ms, real refresh time depends on other functions
        
        
        # AF_cube function closing part
        if timeout==True or robot_stop ==True:       # timeout or robot being stopped
            quit_func(quit_script=False)             # quit function is called, withou forcing the script quitting
            return                                   # cubeAF function is terminated
    
    # AF_cube function closing part
    if timeout==True or robot_stop ==True:           # timeout or robot being stopped
        quit_func(quit_script=False)                 # quit function is called, withou forcing the script quitting
        return                                       # cubeAF function is terminated







if __name__ == "__main__":
    """ This function takes care of few things:
        1) initial settings
        2) some general settings (if to printout debug prints, internet connection / time synchronization, if screen connected, etc)
        3) waits for user to press the button, and it starts the cube reading phase."""
    
    global camera, rawCapture, width, height, robot_stop, robot_idle, timeout, light_program
    
    ################    general settings on how the robot is operated ###############################
    debug = False           # flag to enable/disable the debug related prints
    screen = True           # flag to enable/disable commands requiring a screen connection, for graphical print out
    fixWindPos = True       # flag to fix the CV2 windows position, starting from coordinate 0,0 (top left)
    cv_wow = False          # flag to enable/disable the visualization of the image analysis used to find the facelets
    if cv_wow:              # case the cv image analysis plot is set true
        screen = True       # screen related functions are activated
        fixWindPos = False  # fix position for final image is set false
    fps = False             # flag to check and plot the fps on the frame (more for fun than need)
    
    clear_terminal()                        # cleares the terminal
    print('\nGeneral settings:')            # feedback is printed to the terminal
    if debug:                               # case the debug print-out are requested
        print(f'Debug prints activated\n')  # feedback is printed to the terminal
    else:                                   # case the debug print-out are not requested
        print(f'Debug prints not activated\n') # feedback is printed to the terminal
    
    param_imported, settings = import_parameters() # imports the parameter from a json file
    if not param_imported:                  # case the function import_parameters returns False
        quit_func(quit_script=True)         # qutting function is called, with script clossure
    # ###############################################################################################
    
    
    
    ################    Display setting       ######################################################
    from Cubotino_T_display import display as disp # sets the display object (the one on the robot)             
    disp.clean_display()                    # cleans the display
    # ##############################################################################################
        
    
    
    ################    Python version info    ######################################################
    import sys                              # sys library is imported to check the python version
    try:                                    # tentative approach
        print(f'Python version: {sys.version}')   # print to terminal the python version
    except:                                 # case an exception is raised
        pass                                # no actions
    # ###############################################################################################
    
    
        
    ################    processor version info    ###################################################
    # when armv6 it skips one openCV comand that crash the script when Raspberry Pi Zero (not 2)
    import os                               # os is imported to check the machine
    light_program = False                   # flag of a lighter program (OK on Rpi 3, 4 and Zero2)
    processor = ''                          # processor string variable is set empty
    try:                                    # tentative approach
        processor = os.uname().machine      # processor is inquired
        print(f'Processor architecture: {processor}')  # print to terminal the processor architecture
        if 'armv6' in processor:            # case the string armv6 is contained in processor string
            light_program = True            # flag for a lighter program is set true (OK for Rpi Zero)
    except:                                 # case an exception is raised
        pass                                # no actions
    print('light_program: ', light_program) # feedback is printed to the terminal
    # ###############################################################################################

 
    
    ################    screen presence, a pre-requisite for graphical   ############################
    import time
    screen_presence = check_screen_presence()             # checks if a screen is connected (also via VNC)
    
    if debug:                                             # case the debug print-out are requested
        print("screen_presence: ",screen_presence)        # feedback is printed to the terminal
    
    if not screen_presence:                               # case there is not a screen connected 
        print(f'Screen related function are not activated')   # feedback is printed to the terminal 
        debug = False                                     # debug flag is set false
        disp.set_backlight(1)                             # display backlight is turned on, in case it wasn't
        disp.show_on_display('EXT. SCREEN', 'ABSENCE', fs1=16, fs2=20)  #feedbak is print to to the display
        time.sleep(2)                                     # time delay to let possible readig the screen
        screen = False                                    # screen flag is forced false
        fps = False                                       # fps flag is forced false
    
    if screen_presence:                                   # case there is a screen connected 
        disp.set_backlight(1)                             # display backlight is turned on, in case it wasn't
        disp.show_on_display('EXT. SCREEN', 'PRESENCE', fs1=16, fs2=19 )  #feedbak is print to to the display
        time.sleep(2)
        print(f'Screen related function are activated')   # feedback is printed to the terminal 
        if fixWindPos:                                    # case the graphical windows is forced to the top left monitor corner
            print(f'CV2 windows forced to top-left screen corner')    # feedback is printed to the terminal     
        
        if cv_wow:                                        # case the cv image analysis plot is set true
            print(f'cv image analysis is plot on screen') # feedback is printed to the terminal 
        
        if fps:                                           # case the fps flas is set true
            print(f'FPS calculation and plot on frame are activated')      # feedback is printed to the terminal 
        else:                                             # case the fps flas is set true
            print(f'FPS calculation and plot on frame are not activated')  # feedback is printed to the terminal
        
        if frameless_cube == 'false':                               # case the frameless string variale equals to false
            print('\nCube status detection set for cube with black frame around the facelets')  # feedback is printed to the terminal 
        elif frameless_cube == 'true':                              # case the frameless string variale equals to true:                                                                      # case the frameless flag is set true
            print('\nCube status detection set for frameless cube') # feedback is printed to the terminal
        elif frameless_cube == 'auto':                              # case the frameless string variale equals to true:                                                                      # case the frameless flag is set true
            print('\nCube status detection set for both cubes with and without black frame')   # feedback is printed to the terminal 
            print('This setting takes slightly longer time for the cube status detection\n')   # feedback is printed to the terminal 
    # ###############################################################################################
    
    
    
    ###################################    import libraries    ######################################
    print('\nimport libraries:')            # feedback is printed to the terminal    
    import_libraries()                      # imports libraries
    # ###############################################################################################
    
    
    
    #################################    startup  variables     #####################################
    print('\nother settings and environment status:')  # feedback is printed to the terminal                
    start_up(first_cycle = True)            # sets the initial variables, in this case it is the first cycle
    reset_camera = False                    # at the start the camera has default settings, no need to reset it
    solv_cycle = 0                          # variable to count the solving cycles per session is set to zero
    scramb_cycle = 0                        # variable to count the scrambling cycles per session is set to zero
    print('\n#############################################################################\n')
    # ###############################################################################################
    
    
    
    ###################### pre-setting a number of scrambling and solving cycles   ##################
    if args.cycles != None:                 # case the Cubotino_T.py has been launched with 'cycles' argument
        cycles_num = abs(int(args.cycles))  # the positive integer arg passed is assigned to the cycle_num variable
        if cycles_num > 0:                  # case the automated cycles request is more than zero
            print(f'\nAsked the robot to automatically scramble and solve the cube {cycles_num} times')
            automated = True                # automated variable is set true
        else:                               # case the automated cycles request equals zero
            automated = False               # automated variable is set false
    else:                                   # case the Cubotino_T.py has not been launched without 'cycles' argument
        cycles_num = 0                      # zero is assigned to the cycles_num variable
        automated = False                   # automated variable is set false
    
    if cycles_num > 0 and args.pause != None:  # case the Cubotino_T.py has been launched also with 'pause' argument
        cycle_pause = abs(int(args.pause))  # the positive integer arg passed is assigned to the cycle_pause variable
        print(f'Asked the robot to pause {cycle_pause} seconds in between the automated cycles\n') 
    else:                                   # case the Cubotino_T.py has not been launched without 'pause' argument
        cycle_pause = 0                     # zero is assigned to the cycles_num variable
    # ###############################################################################################
        
    

    while True:                             # (outer) infinite loop, until the Rpi is shut-off
        robot_stop = False                  # flag used to stop or allow robot movements
        robot_idle = True                   # robot is idling
        timeout = False                     # flag used to limit the cube facelet detection time
        
        if reset_camera:                    # case the reset_camera is set true (after a solving cycle is done or stopped)
            close_camera()                  # this is necessary to get rid of analog/digital gains previously used
            time.sleep(0.5)                 # little delay between the camera closing, and a new camera opening
            camera, rawCapture, width, height = webcam()  # camera has to be re-initialized, to removes previous settings
            start_up(first_cycle = False)   # sets the initial variables, in this case it is not the first cycle
        
        
        while not automated:                # while automated is false: (inner) infinite loop, until 'solve' process is chosen
            print('\n\n\n\nWaiting for user to start a cycle')  # feedback is printed to the terminal
            cycle = press_to_start()        # request user to press the button to start a scrambling or solving cycle
            
            if cycle == 'scramble':         # case the chosen cycle is cube scrambling
                # scramble can be done more times within this inner while loop
                scramb_cycle += 1           # counter, for the number of scrambling cycles perfomed within a session, is incremented
                start_scrambling(scramb_cycle)  # start_scrambling function is called
            
            elif cycle == 'solve':          # case the chosen cycle is cube scrambling
                solv_cycle += 1             # counter, for the number of solving cycles perfomed within a session, is incremented
                reset_camera = start_solving(solv_cycle)  # start_solving function is called
                break      # (inner) infinite loop is interrupted once cube solving cycle is done or stopped
      
        if automated:                           # case automated variable is true
            for i in range(cycles_num):         # iteration over the number passed to the --cycles argument 
                start_automated_cycle(i+1, cycles_num, cycle_pause)  # start_automated_cycle finction is called
                
                if i+1 < cycles_num:            # case there is at least one more cycle to do
                    # preparing the camera and variables for the next solving cycle
                    robot_stop = False          # flag used to stop or allow robot movements
                    robot_idle = True           # robot is idling
                    timeout = False             # flag used to limit the cube facelet detection time       
                    close_camera()              # this is necessary to get rid of analog/digital gains previously used
                    time.sleep(0.5)             # little delay between the camera closing, and a new camera opening
                    camera, rawCapture, width, height = webcam()  # camera has to be re-initialized, to removes previous settings
                    start_up(first_cycle = False)  # sets the initial variables, in this case it is not the first cycle
                
                if i+1 == cycles_num:           # case all the cycles have been done
                    # closing the automated cycles section
                    print('\n\n\n\n\n\n')       # prints some empty lines, on IDE terminal
                    clear_terminal()            # cleares the terminal
                    print(f'\nScrambled and solved the cube {cycles_num} times\n')
                    solv_cycle = cycles_num     # cycle_num is assigned to variable counting the solving cycles manually requested
                    scramb_cycle = cycles_num   # cycle_num is assigned to variable counting the scrambling cycles manually requested
                    reset_camera = True         # camera reset for the next (non-automated) cycle



                    # below boolean sets what the robot does after the automated cycles are done                  
                    switch_off_after_automated_cycles = False  
                    
                    if switch_off_after_automated_cycles:  # case switch_off_after_automated_cycles is set True
                        quit_func(quit_script=True)        # script is quitted and Rpi is shut off
                    else:                                  # case switch_off_after_automated_cycles is set False
                        automated = False                  # automated variable is set false
                        # the robot will idle while waiting for command at the touch button
                    
                   

                
