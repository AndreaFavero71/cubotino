#!/usr/bin/python
# coding: utf-8

""" 
#############################################################################################################
#  Andrea Favero, 28 January 2024
#
#  This code relates to CUBOTino autonomous, a very small and simple Rubik's cube solver robot 3D printed.
#  CUBOTino autonomous is the 'Top version', of the CUBOTino robot series.
#  Demo at https://youtu.be/dEOLhvVMcUg
#  Instructions:https://www.instructables.com/CUBOTino-Autonomous-Small-3D-Printed-Rubiks-Cube-R/
#
#  This is the core script, that interacts with few other files.
#  Many functions of this code have been developed since 2021, for my previous robots (https://youtu.be/oYRXe4NyJqs).
#
#  The cube status is detected via a camera system (PiCamera) and OpenCV .
#  Kociemba solver is used foer the cube solution (from: https://github.com/hkociemba/RubiksCube-TwophaseSolver)
#  Credits to Mr. Kociemba for his great job !
#  Search for CUBOTino autonomous on www.instructables.com, to find more info about this robot.
#
#  Developped on:
#  - Raspberry Pi Zero2W and Raspberry Pi ZeroW
#  - Raspberry Pi OS 10 (A port of Debian Buster with security updates and desktop environment)
#    (Linux raspberry 5.10.103-v7+ #1529 SMP Tue 8 12:21:37 GMT 2022 armv71 GNU/Linux)
#  - OpenCV cv2 ver: 4.1.0
#  - PiCamera v1.3
#  - Numpy: 1.21.4
#
#  Updated for:
#  - Raspberry Pi OS 11 (A port of Debian Bulleys with security updates and desktop environment)
#
#############################################################################################################
"""


# __version__ variable
version = '6.5 (28 Jan 2024)'


################  setting argparser for robot remote usage, and other settings  #################
import argparse

# argument parser object creation
parser = argparse.ArgumentParser(description='CLI arguments for Cubotino_T.py')

# --version argument is added to the parser
parser.add_argument("-v", "--version", help='Shows the script version.', action='version',
                    version=f'%(prog)s ver:{version}')

# --debug argument is added to the parser
parser.add_argument("-f", "--twosteps", action='store_true',
                    help="From Flip-Up to close_cover in 2 movements instead of 1")

# --debug argument is added to the parser
parser.add_argument("-d", "--debug", action='store_true',
                    help="Activates printout of settings, variables and info for debug purpose")

# --cv_wow argument is added to the parser
parser.add_argument("--cv_wow", action='store_true',
                    help="Activates the cv_wow (image analysis steps) on screen")

# --F_deg argument is added to the parser
parser.add_argument("-F", "--F_deg", action='store_true',
                    help="CPU temperature in Fahrenheit degrees")

# --cycles argument is added to the parser
parser.add_argument("-c", "--cycles", type=int, 
                    help="Input the number of automated scrambling and solving cycles")

# --pause argument is added to the parser
parser.add_argument("-p", "--pause", type=int, 
                    help="Input the pause time, in secs, between automated cycles")

# --shutoff argument is added to the parser
parser.add_argument("-s", "--shutoff", action='store_true',
                    help="Shuts the RPI off, after the automated scrambling and solving cycles")

# --timer argument is added to the parser
parser.add_argument("-t", "--timer", action='store_true',
                    help="Timer is visualized after the scrambling function (max 1h)")

# --picamera_test argument is added to the parser
parser.add_argument("--picamera_test", action='store_true',
                    help="Visualizes PiCamera images for test purpose")

# --no_btn argument is added to the parser
parser.add_argument("--no_btn", action='store_true',
                    help="Starts the first solving cycles without using the touch button")

# --silent is added to the parser
parser.add_argument("--silent", action='store_true',
                    help="Deactivate servos, for 'quite' debug of non-servos related aspects")
args = parser.parse_args()   # argument parsed assignement
# ###############################################################################################


import sys                                               # sys library is imported
from get_macs_AF import get_macs_AF                      # import the get_macs_AF function





def get_fname_AF(fname, pos):
    """generates a filename based on fname and pos value.
        This is used to match AF specific setting files to the mac address and its position on macs_AF.txt"""
    return fname[:-4]+'_AF'+str(pos+1)+'.txt'





def import_parameters(debug=False):
    """ Function to import parameters from a json file, to make easier to list/document/change the variables
        that are expected to vary on each robot."""
       
    global frameless_cube, camera_width_res, camera_hight_res, s_mode
    global expo_shift, kl, x_l, x_r, y_u, y_b, w_f, w_s, square_ratio, rhombus_ratio
    global delta_area_limit, sv_max_moves, sv_max_time, collage_w, marg_coef, cam_led_bright, cam_led_auto
    global detect_timeout, show_time, warn_time, quit_time, cover_self_close, vnc_delay, fcs_delay
    global built_by, built_by_x, built_by_fs
    global pathlib, json
    
    # convenient choice for Andrea Favero, to upload the settings fitting my robot, via mac address check
    import os.path, pathlib, json                                 # libraries needed for the json, and parameter import
    from getmac import get_mac_address                            # library to get the device MAC ddress
    import time
    
    macs_AF = get_macs_AF()                                       # mac addresses of AF bots are retrieved
    fname = 'Cubotino_T_settings.txt'                             # fname for the text file to retrieve settings
    folder = pathlib.Path().resolve()                             # active folder (should be home/pi/cubotino/src)  
    eth_mac = get_mac_address().lower()                           # mac address is retrieved
    if eth_mac in macs_AF:                                        # case the script is running on AF (Andrea Favero) robot
        pos = macs_AF.index(eth_mac)                              # return the mac addreess position in the tupple
        fname = get_fname_AF(fname, pos)                          # generates the AF filename
    else:                                                         # case the script is not running on AF (Andrea Favero) robot
        fname = os.path.join(folder, fname)                       # folder and file name for the settings, to be tuned
    
    if os.path.exists(fname):                                     # case the settings file exists
        with open(fname, "r") as f:                               # settings file is opened in reading mode
            settings = json.load(f)                               # json file is parsed to a local dict variable
            # NOTE: in case of git pull, the settings file will be overwritten, the backup file not
        
        # update key-values parameters, needed in case of additional parameters added at remote repo
        settings = update_settings_file(fname, settings)
        
        if debug:                                                 # case debug variable is set true on __main_
            print('\nImporting settings from the text file:', fname)    # feedback is printed to the terminal
            print('\nImported parameters: ')                      # feedback is printed to the terminal
            for parameter, setting in settings.items():           # iteration over the settings dict
                print(parameter,': ', setting)                    # feedback is printed to the terminal
            print()
            
        os.umask(0) # The default umask is 0o22 which turns off write permission of group and others
        backup_fname = os.path.join(folder,'Cubotino_T_settings_backup.txt')          # folder and file name for the settings backup
        with open(os.open(backup_fname, os.O_CREAT | os.O_WRONLY, 0o777), 'w') as f:  # settings_backup file is opened in writing mode
            if debug:                                                 # case debug variable is set true on __main_
                print('Copy of settings parameter is saved as backup at: ', backup_fname)    # feedback is printed to the terminal
            f.write(json.dumps(settings, indent=0))                   # content of the setting file is saved in another file, as backup
            # NOTE: in case of git pull, the settings file will be overwritten, the backup file not

        try:                                                          # tentative
            if settings['frameless_cube'].lower().strip() == 'false': # case frameless_cube parameter is a string == false
                frameless_cube = 'false'                              # cube with black frame around the facelets
            elif settings['frameless_cube'].lower().strip() == 'true': # case frameless_cube parameter is a string == true
                frameless_cube = 'true'                               # cube without black frame around the facelets
            elif settings['frameless_cube'].lower().strip() == 'auto': # case frameless_cube parameter is a string == auto
                frameless_cube = 'auto'                               # cube with/without black frame around the facelets
            else:                                                     # case the frameless parameter is not 'false', 'true' or 'auto'
                print('\n\nAttention: Wrong frameless_cube parameter: It should be "true", "false" or "auto".\n')  # feedback is printed to the terminal
                frameless_cube = 'auto'                               # cube with/without black frame around the facelets
            
            expo_shift = float(settings['expo_shift'])                # camera exposure shift
            kl = float(settings['kl'])                                # coefficient for PiCamera stability acceptance (l= lower)
            x_l = int(settings['x_l'])                                # image crop on left (before warping)
            x_r = int(settings['x_r'])                                # image crop on right (before warping)
            y_u = int(settings['y_u'])                                # image crop on top (before warping)
            y_b = int(settings['y_b'])                                # image crop on bottom (before warping)
            w_f = float(settings['warp_fraction'])                    # coeff for warping the image
            w_s = float(settings['warp_slicing'])                     # coeff for cropping the bottom warped image
            if w_s == 0:                                              # case the parameter equals to zero
                w_s = 0.1                                             # the parameter is set to 0.1
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
            warn_time = float(settings['warn_time'])                  # solve button pressing time before get a worning
            quit_time = float(settings['quit_time'])                  # solve button pressing time before the quit process
            vnc_delay = float(settings['vnc_delay'])                  # delay for cube moving to next face at scan (for VNC viewer)
            built_by = str(settings['built_by'])                      # maker's name to add on the Cubotino logo
            built_by_x = int(settings['built_by_x'])                  # x coordinate for maker's name on display
            built_by_fs = int(settings['built_by_fs'])                # font size for the maker's name on display
            fcs_delay = float(settings['fcs_delay'])                  # delay in secs to switch to Fix Coordinates System for facelets position
            if settings['cover_self_close'].lower().strip() == 'false':  # case cover_self_close parameter is a string == false
                cover_self_close = False                              # cover_self_close parameter is set boolean False
            elif settings['cover_self_close'].lower().strip() == 'true': # case cover_self_close parameter is a string == true
                cover_self_close = True                               # cover_self_close parameter is set boolean True
            else:                               # case the frameless parameter is not 'false', 'true' or 'auto'
                print('\n\nAttention: Wrong cover_self_close parameter: It should be "true" or "false."\n')  # feedback is printed to the terminal
                cover_self_close = False                              # cover_self_close parameter is set boolean False
            return True, settings
            
        except:   # exception will be raised if json keys differs, or parameters cannot be converted to int/float
            print('Error on converting the imported parameters to int, float or string')   # feedback is printed to the terminal                                  
            return False, ''                                    # return robot_init_status variable, that is False
    
    else:                                                       # case the settings file does not exists, or name differs
        print('Could not find Cubotino_T_servo_settings.txt')   # feedback is printed to the terminal                                  
        return False, ''                                        # return robot_init_status variable, that is False







def update_settings_file(fname, settings):
    """Function to check if the existing fname (Cubotino_T_settings.txt) has all the parameters that were instroduced after firt release.
        This will allow every Makers to 'git pull' the updates while pre-serving personal settings.
        This works by adding  *_settings.txt to gitignore.
        Cubotino_T.py will update Cubotino_T_settings.txt with eventually missed parameters and related default values.
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
        settings['expo_shift']='-0.5'        # this parameter is ignored on OS10 (Buster)
        any_change = True
    
    if 'fcs_delay' not in settings_keys:
        settings['fcs_delay']='3'
        any_change = True
     
    if any_change:
        print('\nOne time action: Adding new parameters to the Cubotino_T_settings.txt')
        print('Action necessary for compatibility with the latest downloaded Cubotino_T.py \n')
        with open(fname, 'w') as f:
            f.write(json.dumps(settings, indent=0))   # content of the updated setting is saved
    
    return settings







def import_libraries():
    """ Import of the needed libraries.
        These librries are imported after those needed for the display management.
        Kociemba solver is tentatively imported considering three installation/copy methods."""
    
    global servo, rm, Popen, PIPE, camera, GPIO, median, dt, sv, cubie, np, math, time, cv2, os
    
    
    # import custom libraries
    import Cubotino_T_servos as servo                     # custom library controlling Cubotino servos and led module
    import Cubotino_T_moves as rm                         # custom library, traslates the cuber solution string in robot movements string

    # import non-custom libraries
    from statistics import median                         # median is used as sanity check while evaluating facelets contours
    from subprocess import Popen, PIPE                    # module for interacting with some shell commands
    import RPi.GPIO as GPIO                               # import RPi GPIO library
    import datetime as dt                                 # mainly used as timestamp, like on data logging
    import numpy as np                                    # data array management
    import math                                           # math package
    import time                                           # time package
    import cv2                                            # computer vision package
    import os                                             # os is imported to ensure the file presence check/make
    
    print(f'CV2 version: {cv2.__version__}')              # print to terminal the cv2 version
    
    # Up to here Cubotino logo is shown on display
    disp.show_on_display('LOADING', 'SOLVER', fs1=24, fs2=27)  # feedback is printed to the display
    disp.set_backlight(1)                                 # display backlight is turned on, in case it wasn't

    
    # importing Kociemba solver
    # this import takes some time to be uploaded
    # there are three import attempts, that considers different solver installation methods
    try:                                                  # attempt
        import solver as sv                               # import Kociemba solver copied in /home/pi/cubotino/src
        import cubie as cubie                             # import cubie Kociemba solver library part
        solver_found = True                               # boolean to track no exception on import the copied solver
        if debug:                                         # case debug variable is set True            
            print('Found Kociemba solver copied in active folder')   # feedback is printed to the terminal
    except:                                               # exception is raised if no library in folder or other issues
        solver_found = False                              # boolean to track exception on importing the copied solver
       
    if not solver_found:                                  # case the solver library is not in active folder
        import os.path, pathlib                           # library to check file presence            
        folder = pathlib.Path().resolve()                 # active folder (should be home/pi/cubotino/src)  
        fname = os.path.join(folder,'twophase','solver.py')   # active folder + twophase' + solver name
        if os.path.exists(fname):                         # case the solver exists in 'twophase' subfolder
            try:                                          # attempt
                import twophase.solver as sv              # import Kociemba solver copied in twophase sub-folder
                import twophase.cubie as cubie            # import cubie Kociemba solver library part
                solver_found = True                       # boolean to track the  import the copied solver in sub-folder
                if debug:                                 # case debug variable is set True            
                    print('Found Kociemba solver copied in subfolder')  # feedback is printed to the terminal
            except:                                       # exception is raised if no library in folder or other issues
                pass                                      # pass, to keep solver_found = False as it was before
        
    if not solver_found:                                  # case the library is not in folder or twophase' sub-folder
        try:                                              # attempt
            import twophase.solver as sv                  # import Kociemba solver installed in venv
            import twophase.cubie as cubie                # import cubie Kociemba solver library part
            twophase_solver_found = True                  # boolean to track no exception on import the installed solver
            if debug:                                     # case debug variable is set True
                print('Found Kociemba solver installed')  # feedback is printed to the terminal
        except:                                           # exception is raised if no library in venv or other issues
            twophase_solver_found = False                 # boolean to track exception on importing the installed solver
    

    if not solver_found and not twophase_solver_found:    # case no one solver has been imported
        print('\nNot found Kociemba solver')              # feedback is printed to the terminal
        disp.show_on_display('NO SOLVER', 'FOUND', fs1=19, fs2=28) # feedback is printed to the display
        time.sleep(5)                                     # delay to let user the time to read the display
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
                disp.show_cubotino(built_by, built_by_x, built_by_fs)  # shows the Cubotino logo on the display, for delay time
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


##### addition for Maker Faire setup ##########
# it uses two inputs in logic AND to start the robot. These buttons don't do enything else (no scrambling, no time filtering)
        if GPIO.input(touch_btn1_faire) and GPIO.input(touch_btn2_faire):
            # case both the touch buttons on the faire setup are 'pressed'
            return 'solve'  # 'solve' is returned
###############################################





def solve_or_scramble():
    """ Sense for a single or double touches at the touch sensor, within a time limit.
        Returns 'solve' when single touch and 'scramble' when multiple touches.
        If the button is kept pressed longer, the robot can be restarted or shutted OFF."""
    
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
    
    if 'DISPLAY' in os.environ:                # case the environment variable DISPLAY is set to 1 (there is a display)
        return True                            # function returns true, meaning there is a screen connected
    else:                                      # case the environment variable DISPLAY is set to 0 (there is NOT a display)
        return False                           # function returns false, meaning there is NOT a screen connected







def get_os_version():
    os_version = 0
    with open('/etc/os-release', 'r') as file:
        for row in file:
            if "VERSION_ID" in row:
                os_version = int(''.join(x for x in row if x.isdigit()))
                return os_version
    return os_version







def set_camera(expo_shift=0):
    """ Set the camera and its resolution.
        Different libraries will be used depending on OS version (picamera on OS 10, picamera2 on OS 11)"""
  
    os_version = get_os_version()
    
    if os_version >0 and os_version <= 10:
        from Cubotino_T_camera_os10 import camera
        width = camera.get_width()
        height = camera.get_height()
        
        # case cube side is zero, no request to stop the robot and not automated cycles
        if side == 0 and not robot_stop and cycles_num == 0 and not picamera_test:
            camera.printout()
        
    elif os_version >= 11:
        from Cubotino_T_camera_os11 import camera
        width = camera.get_width()
        height = camera.get_height()
        time.sleep(1)
        camera.set_exposure_shift(expo_shift)
    
    else:
        print("Raspberry Pi OS version not recognized")
    
    return camera, width, height







def robot_camera_setting(debug, os_version, camera, face):
    """ According to PiCamera documentation it is required a couple of seconds to warmup the camera.
    If the robot start-button is pressed right after placing the cube, the camera has to update the gains previously set,
    that means from when the camera was pointing the black cube support; this can take up to 20 secs to get all
    the gains stable (awb gains are rather slow to update).
    Much different is when the cube is on the cube support for few secs when the button is pressed.
    To properly cover all the possible situations, this fuction releases the camera warm-up phase only after
    all the gains are stable. This function returns the camera parameters once within absolute variation < 5% from
    the average of 'pts' points."""
    
    t_start = time.time()     # time reference
    stable_camera = False     # flag to trak whether the camera gets stable within the timeout
    
    
    if os_version <= 10:      # case the OS is Buster or older
        t_max = 5             # timeout to quit this function and deliver the latest camera parameters
        if debug:             # case debug variable is set True
            pts = 8           # consecutive datapoints to analyse if parameters within acceptable range
        else:                 # case debug variable is set False
            pts = 10          # consecutive datapoints to analyse if parameters within acceptable range
        

    elif os_version >= 11:    # case the OS is Bullseye or newer
        t_max = 3             # timeout to quit this function and deliver the latest camera parameters
        if debug:             # case debug variable is set True
            pts = 9           # consecutive datapoints to analyse if parameters within acceptable range
        else:                 # case debug variable is set False
            pts = 11          # consecutive datapoints to analyse if parameters within acceptable range
        
    
    ku = 2-kl                 # Upper koefficient to define acceptance bandwidth (default +/-5%) 

    a_gain_list=[]                                     # list to store the Picamera analog gain, during pre-scan period
    d_gain_list=[]                                     # list to store the Picamera digital gain, during pre-scan period
    awb_blue_list=[]                                   # list to store the Picamera AWB gain, for blue, during pre-scan period
    awb_red_list=[]                                    # list to store the Picamera AWB gain, for red, during pre-scan period
    exp_list=[]                                        # list to store the Picamera exposure time, during pre-scan period
    PiCamera_param=()                                  # empty tuple is assigned to PiCamera_param variable
    
    if screen and not robot_stop:                      # case screen variable is set True
        if fixWindPos:                                 # case the fixWindPos variable is chosen
            cv2.namedWindow('cube')                    # create the cube window
            cv2.moveWindow('cube', 0,0)                # move the window to (0,0)
                
    while time.time()-t_start<t_max:                   # timeout for camera stabilization
        if screen and not robot_stop:                  # case screen variable is set True
            frame, w, h = read_camera()                # camera reading, not necessary but nice to show
            cv2.imshow('cube', frame)                  # shows the frame 
            cv2.waitKey(1)                             # tot refresh time of 1ms (in reality much more)
        else:                                          # case screen variable is set False
            time.sleep(0.11)                           # similar time when there is no camera reading and plot to screen
        
        if os_version <= 10:
            pts = 8   # consecutive datapoints to analyse if parameters within acceptable range
            a_gain, d_gain, awb_gains, exposure = camera.get_metadata()  # camera is inquired
 
        elif os_version >= 11:
            pts = 9   # consecutive datapoints to analyse if parameters within acceptable range
            metadata = camera.get_metadata()           # camera is inquired
            a_gain = metadata["AnalogueGain"]          # analog gain from metadata is assigned to the variable
            d_gain = metadata["DigitalGain"]           # digital gain from metadata is assigned to the variable
            awb_gains = metadata["ColourGains"]        # AWB gains from metadata is assigned to the variable
            exposure = metadata["ExposureTime"]        # exposure time from metadata is assigned to the variable        
        
        a_gain_list.append(float(a_gain))              # analog gain is appended to a list
        d_gain_list.append(float(d_gain))              # digital gain is appended to a list
        awb_blue_list.append(float(awb_gains[0]))      # awb blue part gain is appended to a list
        awb_red_list.append(float(awb_gains[1]))       # awb blue part gain is appended to a list
        exp_list.append(exposure)                      # exposure time (micro secs) is appended to a list

        a_check=False                                  # a flag is negatively set for analog gain (being stable...)
        d_check=False                                  # a flag is negatively set for digital gain (being stable...)
        awb_blue_check=False                           # a flag is negatively set for awb blue gain (being stable...)
        awb_red_check=False                            # a flag is negatively set for awb red gain (being stable...)
        exp_check=False                                # a flag is negatively set for the exposure time (being stable...)

        if len(a_gain_list) > pts:                     # requested a minimum amount of datapoints
            check = a_gain_list[-1]/(sum(a_gain_list[-pts:])/pts)  # last analog gain value is compared to the average of last pts points
            if check > kl and check < ku:              # if comparison within acceptance boundaries
                a_check=True                           # flag is positively set for this gain             
        
            check = awb_blue_list[-1]/(sum(awb_blue_list[-pts:])/pts) # last awb_blue gain is compared to the average of last pts points
            if check > kl and check < ku:              # if comparison within acceptance boundaries
                awb_blue_check=True                    # flag is positively set for this gain    
                
            check = awb_red_list[-1]/(sum(awb_red_list[-pts:])/pts) # last awb_red gain is compared to the average of last pts points
            if check > kl and check < ku:              # if comparison within acceptance boundarie
                awb_red_check=True                     # flag is positively set for this gain
            
            check = exp_list[-1]/(sum(exp_list[-pts:])/pts)  # last exposure time is compared to the average of last pts points
            if check > kl and check < ku:              # if comparison within acceptance boundarie
                exp_check=True                         # flag is positively set for the exposure time                
     
            if a_check and awb_blue_check and awb_red_check and exp_check: # if all gains are stable
                stable_camera = True
                break                                  # camera warmup while loop break
    
    a_gain_value = a_gain_list[-1]  # last a_gain list value is assigned
    d_gain_value = d_gain_list[-1]  # last d_gain list value is assigned
    awb_gains = (awb_blue_list[-1], awb_red_list[-1])  # last awb_gains list tuple is assigned
    exposure = exp_list[-1]         # last exposure list value is assigned
    t_stable = round(time.time()-t_start,1)            # time to get the camera stable on this cube face
    
    
    # latest PiCamera settings, in auto mode, are assigned to a tuple
    PiCamera_param=(a_gain_value, d_gain_value, awb_gains, exposure, stable_camera, t_stable)  
          
    if debug:                                          # case debug variable is set True
        print('\nPiCamera warmup function, face:', face)  # feedback is printed to the terminal
        print('analog_gain_list',a_gain_list)          # feedback is printed to the terminal
        print('digital_gain_list',d_gain_list)         # feedback is printed to the terminal
        print('awb_blue_list',awb_blue_list)           # feedback is printed to the terminal
        print('awb_red_list',awb_red_list)             # feedback is printed to the terminal
        print('camera exp_list', exp_list)             # feedback is printed to the terminal
        print('datapoints:', len(exp_list))            # feedback is printed to the terminal
    
    return PiCamera_param    # tuple with the camera stable parameter is returned






def robot_consistent_camera_images(debug, os_version, camera, start_time):
    """ Picamera is left in set in Auto mode for Exposure and AWB gains, while presenting 4 cube faces.
    These parameters are retrieved from PiCamera after showing each of these first 4 cube faces.
    Picamera is then set to Manual mode by setting the average exposure time and average AWB to the camera.
    This approach prevents the PiCamera to keep adjusting the AWB and Exposure while reading the different cube faces.
    The average values set to the camera mitigates large differences at the cube, i.e. when the cube is already solved."""
    
    if robot_stop:            # case the robot has been requested to stop
        return                # function is terminated
    
    disp.show_on_display('SETTING', 'CAMERA', fs1=23, fs2=21)    # feedback is printed to the display
    
    
    if debug:                                           # case debug variable is set True
        print("\nPiCamera asked to enter the Auto mode")  # feedback is printed to the terminal
    
    if os_version <= 10:                                # case the OS is 10 or earlier
        camera.set_auto(debug)                          # camera is set to automatic mode
    
    elif os_version >= 11:                              # case the OS is 11 or newer
        # AWB mode (AutoWitheBalance) to use when the camera is set to automati mode
        # AwbMode: 0=Auto - any illumant, 1=Tungsten - tungsten lighting, 2=Fluorescent - fluorescent lighting
        # 3=Indoor - indoor illumination, 4=Daylight - daylight illumination, 5=Cloudy - cloudy illumination
        awb_mode = 3          # this is the  mode that seems to work better in Cubotino
        camera.set_auto(debug, awb_mode, expo_shift)    # camera is set to automatic mode, by also passing the chosen AWB mode
    
    if debug:                                           # case debug variable is set True
        print('\nPiCamera: Reading the stable camera parameters on four cube sides')  # feedback is printed to the terminal

    a_gain_list=[]                                      # list to store the Picamera analog gain, during pre-scan period
    d_gain_list=[]                                      # list to store the Picamera digital gain, during pre-scan period
    awb_blue_list=[]                                    # list to store the Picamera AWB gain, for blue, during pre-scan period
    awb_red_list=[]                                     # list to store the Picamera AWB gain, for red, during pre-scan period
    exp_list=[]                                         # list to store the Picamera exposure time, during pre-scan period
    stable_camera_list=[]                               # list to store the Picamera stability reach within timeout
    stab_time_list=[]                                   # list to store the PiCamera stabilization time per face
    
    face = 1
    PiCamera_param = robot_camera_setting(debug, os_version, camera, face)
    a_gain_list.append(PiCamera_param[0])
    d_gain_list.append(PiCamera_param[1])
    awb_blue_list.append(PiCamera_param[2][0])
    awb_red_list.append(PiCamera_param[2][1])
    exp_list.append(PiCamera_param[3])
    stable_camera_list.append(PiCamera_param[4])
    stab_time_list.append(PiCamera_param[5])
    
    
    if debug:                                           # case debug variable is set True
        print("\nPiCamera stable settings (in Auto mode) at face:", face)  # feedback is printed to the terminal
        print(PiCamera_param)                           # feedback is printed to the terminal
        print()

    # PiCamera is inquired on 4 cube sides reachable via a simple cube flip, to later fix an average parameters
    for face in (6,4,3):                                # iterate over the next 3 faces reachable via a single cube flip    
        robot_to_cube_side(1,cam_led_bright)            # flipping the cube, to reach the next side 
        PiCamera_param = robot_camera_setting(debug, os_version, camera, face)
        a_gain_list.append(PiCamera_param[0])
        d_gain_list.append(PiCamera_param[1])
        awb_blue_list.append(PiCamera_param[2][0])
        awb_red_list.append(PiCamera_param[2][1])
        exp_list.append(PiCamera_param[3])
        stable_camera_list.append(PiCamera_param[4])
        stab_time_list.append(PiCamera_param[5])
    
        if debug:                                       # case debug variable is set True
            print(f"\nPiCamera stable settings (Auto mode) at face:", face)  # feedback is printed to the terminal
            print(PiCamera_param)                       # feedback is printed to the terminal
            print()
    
    # setting the camera in manual mode, for consistent images when scanning
    a_gain = float(sum(a_gain_list)/len(a_gain_list))   # average a_gain is calculated and assigned
    d_gain = float(sum(d_gain_list)/len(d_gain_list))   # average d_gain is calculated and assigned
    awb_gains = (float(sum(awb_blue_list)/len(awb_blue_list)), float(sum(awb_red_list)/len(awb_red_list)))  # average awb_gains is calculated and assigned
    shutter_time = int(sum(exp_list)/len(exp_list))     # average exposure time of UBDF faces
    
    camera.set_gains(debug, a_gain, d_gain, awb_gains)  # sets the gains to the PiCamera, for consisent images
    camera.set_exposure(shutter_time)                   # sets the shutter time to the PiCamera, for consistent images
    
    t_ref = time.time()                                 # current time is assigned to a reference variable
    while time.time()-t_ref < 2:                        # while loop for max 2 seconds
        exposure = camera.get_exposure()                # read picamera exposure time (microsec)
        if abs(exposure-shutter_time) < 0.05*shutter_time:  # case the camera shutter time deviates less than 5% from target shutter_time
            break                                       # while loop is interrupted       
    
    print(f'\nPiCamera defined and frozen parameters (gains, shutter time) in: {round(time.time()-start_time,1)} secs')
    print()
        
    if debug:                                           # case debug variable is set True
        print('\nPiCamera setting up function is terminated')  # feedback is printed to the terminal
        print('PiCamera stabilization time per face', stab_time_list) # feedback is printed to the terminal
        print('PiCamera stable within timeout', stable_camera_list)   # feedback is printed to the terminal
        
        print('\n\nPiCamera average parameters on the 4 faces:')   # feedback is printed to the terminal
        print('Analog_gain_list',a_gain_list)           # feedback is printed to the terminal
        print('Digital_gain_list',d_gain_list)          # feedback is printed to the terminal
        print('Awb_blue_list',awb_blue_list)            # feedback is printed to the terminal
        print('Awb_red_list',awb_red_list)              # feedback is printed to the terminal
        print('Exposure time on UBDF faces : ', exp_list)  # feedback is printed to the terminal
        
        print('\n\nPiCamera average parameters set for consistent images:')   # feedback is printed to the terminal
        print('Analog_gain sent to PiCamera', a_gain)   # feedback is printed to the terminal
        print('Digital_gain sent to PiCamera', d_gain)  # feedback is printed to the terminal
        print('Awb_blue sentt to PiCamera', awb_gains[0]) # feedback is printed to the terminal
        print('Awb_red sent to PiCamera',awb_gains[1])  # feedback is printed to the terminal
        print('Average exposure time: ', int(sum(exp_list)/len(exp_list)), 'micro secs') # feedback is printed to the terminal
        print('Shutter_time set by PiCamera: ', camera.get_exposure(), ' micro secs')  # feedback is printed to the terminal
        print('\n\n\n')                                 # print some separation lines

    disp.clean_display()                                # cleans the display
    robot_to_cube_side(1,cam_led_bright)                # flipping the cube, to reach the 1st face for the scanning process







def read_camera():
    """ Returns the camera reading, and dimensions."""
    
    global previous_time
    
    frame = camera.get_frame()                                    # single frame array request to camera Class
    width = camera.get_width()                                    # camera width request to camera Class
    height = camera.get_height()                                  # camera height request to camera Class
    
    if len(frame)==0:                                             # case the frame is empty
        print('camera frame not available')                       # feedback is print to the terminal
    
    else:                                                         # case the frame is not empty
        if not picamera_test:                                     # case picamera_test is false
            frame, w, h = frame_cropping(frame, width, height, x_l, x_r, y_u, y_b)  # frame is cropped in order to limit the image area to analyze
            frame, w, h = warp_image(frame, w, h, w_f, w_s)       # frame is warped to have a top like view toward the top cube face
            scale = 0.75 if cv_wow else 0.8                       # scaling factor according to cv_wow i 
            frame, w, h = frame_resize(frame, w, h, scale=scale)  # frame is resized (to smaller size), to gain some speed
        
        elif picamera_test:                                       # case picamera_test is True (also when servos_GUI script is used):
            w = width                                             # widht is assigned to w
            h = height                                            # height is assigned to h
        
        return frame, w, h





            
def frame_cropping(frame, width, height, x_l, x_r, y_u, y_b):
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







def warp_image(frame, w, h, w_f, w_s):
    """ Warp the image to remove perspective and simulate a top like view.
    This because PiCamera is at an angle with reference to the top cube face.
    This allows to analyse the facelets contours (square like, area, etc) in a more precise way.
    This also allows a much nicer cube faces collage."""
    
#     NOTE: uncomment below row to remove the image warping, useful for initial setting the camera frame cropping (frame_cropping)
#     return frame, w, h
    
    if cv_wow:                        # case the cv image analysis plot is set true
        global pre_warp, after_warp   # global variable for the frame content, before and after warping
        pre_warp = frame              # frame is assigned to the pre_warp variable
        ww = w                        # frame width before warping is assigned to a local variable
        hh = h                        # frame height before warping is assigned to a local variable
    
    grid_vertices = np.float32([[0,0], [h,0], [h,w], [0,w]])  # original frame vertices

#     w_f = 7           #(AF 7) fractional frame width part to be 'removed' on top left and right of the frame    
    d_x = int(w/w_f)            # pixels to 'remove' on top left and top righ sides of frame, to warp the image
    straight = 1+d_x/h          # corrects the cube face deformation 

    warped_vertices = np.float32([[d_x,0], [h,0], [int(straight*h),w], [-d_x, w]])  # frame coordinates for the transformation matrix
    matrix = cv2.getPerspectiveTransform(warped_vertices, grid_vertices)            # compute perspective matrix

    # do perspective transformation, by adding black pixels where needed
    frame = cv2.warpPerspective(frame, matrix, (max(w,h), max(w,h)), cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, borderValue=(0,0,0))
    
#     w_s = 1.5                   #(AF 1.5)  # Fractional part of the warping portion to be used as slicing from frame bottom
    frame = frame[: -d_x, :-int(d_x/w_s)]   # frame slicing to remove part of the added (black) pixels on the right frame side 
    h, w = frame.shape[:2]                  # new frame height and width
    
    if cv_wow:                                           # case the cv image analysis plot is set true
        pre_warp, _, _ = frame_resize(pre_warp, ww, hh)  # re-sized frame is assigned to the pre_warp global variable
        after_warp, _, _ = frame_resize(frame, w, h)     # re-sized frame is assigned to the after_warp global variable
    
    return frame, w, h







def frame_resize(frame_w, ww, hh, scale=0.8):        
    """ Re-sizes the image after the cropping and warping steps, by a scaling factor.
    This is useful to lower the amount of handled data for the facelects and color detection."""
    
    ww = int(ww * scale)                  # new frame width
    hh = int(hh * scale)                  # new frame height
    
    if Rpi_ZeroW:                         # case Rpi_ZeroW is True (armv6 processor)
        interp_method = cv2.INTER_LINEAR  # bilinear interpolation method (prevents crashing with armv6 processor))
    else:                                 # case Rpi_ZeroW is False (not an armv6 processor)
        interp_method = cv2.INTER_AREA    # interpolation method resamples using pizel area relation

    frame_s = cv2.resize(frame_w, (ww, hh), interpolation = interp_method)  # resized frame
    return frame_s, ww, hh







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
    
    gap_w = 80       # horizontal gap (width) for all windows from 'Pre_warp' window
    gap_h = 100      # vertical gap (height), above first row of images and between the two rows of windows
    
    # note: for precise window position, after windows name at cv2.namedWindow, the parameter cv2.WINDOW_NORMAL  
    # should be used instead of the cv2.WINDOW_RESIZE (default), but image quality on screen changes too much....
    cv2.namedWindow('Pre_warp')                      # create the Pre_warp window
    cv2.moveWindow('Pre_warp', w, gap_h)             # move the Pre_warp window to coordinate
    cv2.namedWindow('After_warp')                    # create the Frame window
    cv2.moveWindow('After_warp', 2*w+gap_w, gap_h)   # move the Frame window to coordinate
    cv2.namedWindow('Gray')                          # create the Gray window
    cv2.moveWindow('Gray', 3*w+gap_w, gap_h)         # move the Gray window to coordinate
    cv2.namedWindow('blurred')                       # create the Blurred window
    cv2.moveWindow('blurred', 4*w+gap_w, gap_h)      # move the Blurred window to coordinate
    cv2.namedWindow('Canny')                         # create the Canny window
    cv2.moveWindow('Canny', w+gap_w, h+2*gap_h)      # move the Canny window to coordinate
    cv2.namedWindow('Dilated')                       # create the Dilated window
    cv2.moveWindow('Dilated', 2*w+gap_w, h+2*gap_h)  # move the Dilated window to coordinate
    cv2.namedWindow('Eroded')                        # create the Eroded window
    cv2.moveWindow('Eroded', 3*w+gap_w, h+2*gap_h)   # move the Eroded window to coordinate
    cv2.namedWindow('Cube')                          # create the Cube window
    cv2.moveWindow('Cube', 4*w+gap_w, h+2*gap_h)     # move the Cube window to coordinate
    
    cv2.imshow("Pre_warp", pre_warp)                 # pre-warp is shown, on a window called Pre_warp
    cv2.imshow("After_warp", after_warp)             # after_warp is shown, on a window called After_warp
    cv2.imshow("Gray", gray)                         # gray is shown, on a window called Gray
    cv2.imshow("blurred", blurred)                   # blurred is shown, on a window called Blurred
    cv2.imshow("Canny", canny)                       # canny is shown, on a window called Canny
    cv2.imshow("Dilated", dilated)                   # dilated is shown, on a window called Dilated
    cv2.imshow("Eroded", eroded)                     # eroded is shown, on a window called Eroded
    cv2.imshow("Cube", cube)                         # cube is shown, on a window called Cube
    
    if side == 1:                   # case the first cube side is under analysis 
        cv2.waitKey(time + 4000)    # all the openCV windows are showed, with an extra time on first cube side
    else:                           # case the cube side under analysis is not the first side
        cv2.waitKey(time)           # all the openCV windows are showed
    
    save_images = False                                         # boolean to enable/disable saving cv_wow images
    folder = pathlib.Path().resolve()                           # active folder (should be home/pi/cubotino/src)  
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
                print('Error in image list')            # feedback is printed to the terminal








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
        if debug:                                 # case debug variable is set True
            print()                               # print an empty line to the terminal
        print(f'Reading side {sides[side]}')      # feedback is printed to the terminal
        prev_side=side                            # current side is assigned to previous side variable

    image, w, h = edge_analysis(frame, w, h)      # image edges analysis is applied to the frame
    
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







def get_facelets(facelets, frame, contour, hierarchy, t_ref):
    """ Contours are analyzed in order to detect the cube's facelets; Argument are simplified contours.
    Returns contours having square characteristics
    
    [parameter to be square like: Area within limits, limited egde lenght deviation, limited diagonals lenght deviation
    (to prevent rhonbus), limited area variation between the 9 facelets].
    This function is called on each of the cube sides, therefore the return relates to the face under analysis.""" 
    
    if len(f_coordinates)>0 and time.time() - t_ref > fcs_delay:  # case the (edges based) facelets detection takes more than fcs_delay secs
        facelets, frame = get_facelets_fcs(facelets, frame)  # facelets info are based on fix coordinates
        fix_c = True                # fix_c (fix coordinates usage) is set to true   
        return facelets, frame, fix_c   # list of potential facelet's contour is returned
        

    min_area = int(0.08*(w*h)/9)    #(AF int(0.08*(w*h)/9))  # min area limit for a single facelet's contour
    max_area = 6*min_area           #(AF 6*min_area)         # max area limit for a single facelet's contour       

#     square_ratio=1       #(AF 1)    # it is like a square when min side/max side < square_ratio (0==perfect square, 1 is rather permissive)
#     rhombus_ratio=0.3    #(AF 0.3)  # considered like a square when diagonal1/diagonal2 > rhombus_ratio (1==perfect, 0.3 is rather permissive)
         
    area = cv2.contourArea(contour)                                       # area of each passed contour is retrieved
    
    if min_area < area < max_area:                                        # filter out too small and too large contours (areas)
        contour_squeeze = np.squeeze(contour)                             # flattens out the list of list used by contours
        edges_delta, axes_ratio = square_check(contour_squeeze)           # sanity check on square and ronbhus shapes
        if edges_delta < square_ratio and axes_ratio > rhombus_ratio:     # check if the contour looks like a square
            cont, in_cont, out_cont = order_4points(contour_squeeze, w, h)  # vertex of each contour are ordered CW from top left
            contour_tmp = [cont]                                          # list is made with the ordered detected contour
            frame = cv2.drawContours(frame, contour_tmp, -1, (255, 255, 255), 1)  # a white polyline is drawn on the contour (1 px thickness)
            if debug:                                                     # case debug variable is set True
                # a white circle is drawn on the 1st vertex (top left), as visual check of proper vertices ordering
                frame = cv2.circle(frame, (contour_tmp[0][0][0],contour_tmp[0][0][1]), 5, (255, 255, 255), -1)
            contour_tmp = [out_cont]                                      # list is made with the ordered outer contour
            frame = cv2.drawContours(frame, contour_tmp, -1, (0, 0, 0), 1)  # a black polyline is drawn on the outer contour (1 px thickness)
            
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
                        facelets, frame = estimate_facelets(facelets,frame, w, h)  # calls the function to estimate the remaining facelets                                 
        
    fix_c = False                   # fix_coordinate approach was not used
    return facelets, frame, fix_c   # list of potential facelet's contour is returned







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
        return facelets, frame                                                # function returns the already detected facelets
    
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
        else:                                         # case the facelet contour center x coordinate does not fit columns
            x = 0                                     # zero is assigned
        if cy[i]<y_low+dist:                          # case the facelet contour center y coordinate is on first grid row                          
            y=1                                       # 1 (as row 1) is assigned
        elif cy[i]>y_low+dist and cy[i]<y_high-dist:  # case the facelet contour center y coordinate is on second grid row
            y=2                                       # 2 (as row 2) is assigned
        elif cy[i]>y_high-dist:                       # case the facelet contour center y coordinate is on third grid row
            y=3                                       # 3 (as row 3) is assigned
        else:                                         # case the facelet contour center y coordinate does not fit rows
            y = 0                                     # zero is assigned
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
            print("Error on estimating the missed facelets")  # feedback is printed to the terminal

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
        
        contour_tmp = [pts]                    # list is made with the ordered outer contour
        frame = cv2.drawContours(frame, contour_tmp, -1, (0, 0, 0), 1)        # a black polyline is drawn on the contour (1 px thickness)
        contour_tmp = [outer_pts]              # list is made with the ordered outer contour
        frame = cv2.drawContours(frame, contour_tmp, -1, (255, 255, 255), 1)  # a white polyline is drawn on the outer contour (1 px thickness)

        tmp = {'area': med_a, 'cx': est[i][0], 'cy': est[i][1], 'contour': pts, 'cont_ordered':pts} # dict with relevant contour info
        facelets.append(tmp)                   # estimated facelets relevant info are appended to the detected facelets list
    
    if screen and debug and frameless_cube!=False:                             # case there is a connected screen, debug and Frameless_cube not False
        cv2.line(frame, (x_low+dist,0), (x_low+dist,h), (255, 255, 255), 2)    # vertical line is drawn between first and second row of facelets
        cv2.line(frame, (x_high-dist,0), (x_high-dist,h), (255, 255, 255), 2)  # vertical line is drawn between second and third row of facelets
        cv2.line(frame, (0,y_low+dist), (w,y_low+dist), (255, 255, 255), 2)    # horizontal line is drawn between first and second column of facelets
        cv2.line(frame, (0,y_high-dist), (w,y_high-dist), (255, 255, 255), 2)  # horizontal line is drawn between second and third column of facelets
        
    return facelets, frame     # detected facelets combined with estimated facelets







def get_facelets_fcs(facelets, frame):
    """This function points to fix coordinates at the cube face image, to retrieve the color.
        Dummy contours are made, to visualize them on screen, or saved picture_collage, when debug is true."""
    
    if debug:
        print("\nCalled the fix_coordinate_approach")
        
    c = f_coordinates                          # f_coordinates assigned to a local very short variable name
    
    # x and y distance between the facelets coordinates
    x_dist = (c[2]-c[0] + c[4]-c[2] + c[8]-c[6] + c[10]-c[8] + c[14]-c[12] + c[16]-c[14])/6
    y_dist = (c[7]-c[1] + c[9]-c[3] + c[11]-c[5] + c[13]-c[7] + c[15]-c[9] + c[17]-c[11])/6
    
    s_side_x = int(round(0.6*x_dist/2,0))      # half side dimension, along x, for the estimated contour square
    s_side_y = int(round(0.6*y_dist/2,0))      # half side dimension, along y, for the estimated contour square                    
    
    area = s_side_x*s_side_y*4                 # estimated area
    for i in range(0,18,2):                    # iteration over the 9 facelets
        tl = [c[i] - s_side_x, c[i+1] - s_side_y]  # top left contour coordinate, calculated from the fix coordinate center point
        tr = [c[i] + s_side_x, c[i+1] - s_side_y]  # top right contour coordinate, calculated from the fix coordinate center point
        br = [c[i] + s_side_x, c[i+1] + s_side_y]  # bottom right contour coordinate, calculated from the fix coordinate center point
        bl = [c[i] - s_side_x, c[i+1] + s_side_y]  # bottom left contour coordinate, calculated from the fix coordinate center point
        pts=np.array([tl, tr, br, bl], dtype="int32")  # estimated contour coordinates      
        
        gap=3                                  # pixels gap
        tl[0]=max(tl[0]-gap,0)                 # top left x coordinate, shifted toward the contour outer side
        tl[1]=max(tl[1]-gap,0)                 # top left y coordinate, shifted toward the contour outer side
        tr[0]=min(tr[0]+gap,w)                 # top right x coordinate, shifted toward the contour outer side
        tr[1]=max(tr[1]-gap,0)                 # top right y coordinate, shifted toward the contour outer side
        br[0]=min(br[0]+gap,w)                 # bottom right x coordinate, shifted toward the contour outer side
        br[1]=min(br[1]+gap,h)                 # bottom right y coordinate, shifted toward the contour outer side
        bl[0]=max(bl[0]-gap,0)                 # bottom left x coordinate, shifted toward the contour outer side
        bl[1]=min(bl[1]+gap,h)                 # bottom left y coordinate, shifted toward the contour outer side
        outer_pts=np.array([tl, tr, br, bl], dtype="int32")     # estimated contour coordinates, sligtly shifted toward the contour outer side
        
        contour_tmp = [pts]                    # list is made with the ordered outer contour
        frame = cv2.drawContours(frame, contour_tmp, -1, (0, 0, 0), 1)        # a black polyline is drawn on the contour (1 px thickness)
        contour_tmp = [outer_pts]              # list is made with the ordered outer contour
        frame = cv2.drawContours(frame, contour_tmp, -1, (255, 255, 255), 1)  # a white polyline is drawn on the outer contour (1 px thickness)

        tmp = {'area': area, 'cx': c[i], 'cy': c[i+1], 'contour':pts, 'cont_ordered':pts} # dict with relevant contour info
        facelets.append(tmp)                   # estimated facelets relevant info are appended to the detected facelets list
    
    return facelets, frame     # facelets info, based on fix coordinates, are returned







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
#         print("Contours areas detected:",area_list)             # feedback is printed to the terminal

    area_median = median(area_list)                             # median area values
    for i in range(len(area_list)):                             # iteration over the facelets areas received
        delta_area=abs((area_list[i]-area_median)/area_median)  # area deviation from the median
        if delta_area > delta_area_limit:                       # filter on area deviating more than threshold
            to_exclude.append(i)                                # list of the contours to exclude is populated
            if debug:                                           # case debug variable is set True
                print('Removed contour with area: ',area_list[i], " having delta_area of:",delta_area) # feedback is printed to the terminal

    if debug:                                                   # case debug variable is set True
        if len(to_exclude)==0 and len(area_list)>=9:            # case all the face facelets have been detected
            if side == 1:                                       # case the face is the first one (U)
                print("Acceptable facelets area from min:",min_area, ",  to max:", max_area, ", with max area_delta <=", delta_area_limit)
            print("Median facelets area:", area_median, "\tareas min:", min(area_list), "\tareas_max:", max(area_list))  # feedback is printed to the terminal
            print("Minimum facelet area delta vs median:", round(abs((min(area_list)-area_median)/area_median),2))       # feedback is printed to the terminal
            print("Maximum facelet area delta vs median:", round(abs((max(area_list)-area_median)/area_median),2))       # feedback is printed to the terminal
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
                break                             # inner for loop can be break once the if found and data appended
    return new_center







def save_coordinates(coordinates):
    """Saves the coordinates of the 9 facelets to a text file.
        This action is done when a cubes_status is correctly determined.
        The 9 coordinates are averaged from the 6 faces."""
    
    print("coordinates:", coordinates)
    coordinates = np.array(coordinates)                     # the coordinates list is converted to numpy array 
    avg = coordinates.mean(axis=0)                          # coordinates are averaged by 'columns'
    avg = np.round(avg, decimals=0).astype(int)             # coordinates are first rounded to 0 decimal then converted to integers
    avg = str(list(avg)).replace('[','').replace(']','')+'\n'  # coordinates are converted to list, then to string without parentheses
    
    fname = 'Cubotino_T_coordinates.txt'                    # fname for the text file to retrieve the coordinates
    folder = pathlib.Path().resolve()                       # active folder (should be home/pi/cubotino/src)
    fname = os.path.join(folder, fname)                     # folder and file name for the coordinates, to be saved
    with open (fname, 'a') as f:                            # text file is opened to write on it in 'appending' mode
        f.write(avg)                                        # coordinates are saved to text file







def load_coordinates():
    """Loads the coordinates of the 4 facelets from a text file.
        Checks if all the characters in file are valid, else remove lines with non valid characters."""
    
    lines = []                                              # empty list to store the data from the file, or to return is empty
    historical_data = False                                 # flag to track presence of historical data is set False
    
    fname = 'Cubotino_T_coordinates.txt'                    # fname for the text file to retrieve the coordinates
    folder = pathlib.Path().resolve()                       # active folder (should be home/pi/cubotino/src) 
    fname = os.path.join(folder, fname)                     # folder and file name for the coordinates, to be loaded
    if os.path.exists(fname):                               # case the coordinates file exists
        with open(fname, "r") as f:                         # settings file is opened in reading mode
            lines = f.readlines()                           # all lines are assigned as list to lines variable
    else:                                                   # case the coordinates file does not exist
        print(f"Not found file {fname}")                    # print feedback to the terminal
        print("This file is generated by the robot at the first successfull cycles") # print feedback to the terminal

    corrupted_data = False                                  # corrupted_data is initially set False
    if len(lines)>=1:                                       # case the file has at least one row of data
        lines = [line.replace(' ', '') for line in lines]   # all spaces are removed
        valid_chars = set('0123456789,')                    # set of valid charcters (includes comma)
        for line in lines:                                  # iteration through the lines
            for c in line:                                  # iteration through the characters in line
                if c not in valid_chars:                    # case the character is not valid
                    corrupted_data = True                   # corrupted_data is set True
                    break                                   # inner for_loop s interrupted
            if corrupted_data:                              # case corrupted_data is True
                break                                       # outer loop_is interrupted
       
    if corrupted_data:                                      # case corrupted_data is True
        if debug:                                           # case debug is set true
            print(f"\nNot valid characters in {fname}")     # feedback is printed to the terminal
        new_lines=[]                                        # empty list to store the cleaned lines
        with open(fname, "w") as f:                         # settings file is opened in writing mode
            for line in lines:                              # iteration through the lines
                if not line.isspace():                      # case the line is not empty
                    keep_line=True                          # keep_line variable is initially set True
                    for c in line.strip().strip('\n'):      # iteration through the characters in line
                        if c not in valid_chars:            # case the character is not valid
                            keep_line=False                 # keep_line variable is set False
                            break                           # inner for_loop is interrupted
                    if keep_line:                           # case keep_line variable is (still) True
                        f.write(line)                       # that line is written to text file
                        new_lines.append(line)              # line is appended to lines
        if debug:                                           # case debug is set true
            print(f"The file got repaired")                 # feedback is printed to the terminal
        lines = new_lines                                   # new_lines is assigned to lines
    
    if len(lines)>=1:                                       # case the file has at least one row of data
        historical_data = True                              # flag to track presence or assence of historical data is set True
    
    if debug:                                               # case debug is set true
        print("Loaded facelets coordinates")                # feedback is printed to the terminal
        
    if historical_data:                                     # case there is historical data
        # convert from string to integers and feed a list with coordinates values as tuple
        all_coordinates = []                                # empty list to store all the coordinates (prevoius solves cubes)
        for line in lines:                                  # iterating on each set of coordinates (each cube)
            coordinates=[]                                  # empty list to store the coordinated parsed from text to integers
            for i in range(7):                              # iterating over the 4 facelets of each cube set of coordinates
                val = int(line[: line.find(',')])           # coordinate is retrieved and converted to integer
                line = line[line.find(',')+1:]              # line is sliced for the remaining coordinates
                coordinates.append(val)                     # facelet coordinate is appended to the coordinates list
            coordinates.append(int(line))                   # the last facelet is appended to the coordinates list
            all_coordinates.append(coordinates)             # all the facelet coordinates are appended to all_coordinates list variable
        
        # limits the dataset to latest 5 cube readings saved (latest scans are more relevant, in case of settings changed at robot)
        n = len(all_coordinates)                            # quantity of coordinates sets (cubes) stored
        if n > 5:                                           # case there are more than 5 sets of coordinates (more than 5 cubes' history)
            all_coordinates = all_coordinates[-5:]          # only the last 5 are considered
        if n > 10:                                          # case there are more than 30 sets of coordinates (more than 5 cubes' history)
            remove_old_data(fname)                          # call a function that removes old data from the text file
        
        # average the coordinates from the dataset
        all_coordinates = np.array(all_coordinates)         # the list of lists all_coordinates is changed to a 2d numpy array
        avg = all_coordinates.mean(axis=0)                  # coordinates are averaged by 'columns'
        avg = np.round(avg, decimals=0).astype(int)         # coordinates are first rounded to 0 decimal then converted to integers
        avg = avg.tolist()                                  # coordinates are assigned to a list
        return avg                                          # the average coordinates are returned
    else:                                                   # case there isn't historical data
        return []                                           # an empty list is returned 







def remove_old_data(fname):
    """"Removes excess data from the coordinates file (it keeps the last 5 rows)."""
    
    with open(fname, "r") as f:                             # settings file is opened in reading mode
        lines = f.readlines()                               # all lines are assigned as list to lines variable
        lines = lines[-5:]                                  # the last 5 lines are assigned to a local variable
    
    with open (fname, 'w') as f:                            # text file is opened to write on it (to empty it)
        f.write('')                                         # an empty character is saved
    
    with open (fname, 'a') as f:                            # text file is opened to write on it, in appending mode
        for line in lines:                                  # iteration over the lines variable
            f.write(line)                                   # line is written to file
    
    if debug:                                               # case debug is set true
        print(f"\nRemoved excessive data from {fname}")     # feedback is printed to the terminal







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

    if debug:                                         # case debug variable is set True
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
    if debug:                                                   # case debug variable is set True
        print(f'\nCube_color_sequence: {cube_color_sequence}')  # feedback is printed to the terminal
    
    return cube_status, HSV_detected, cube_color_sequence, HSV_analysis






def retrieve_cube_color_order(VS_value,Hue):
    """ Determines the cube colors order, meaning the cube's orientation as it has been dropped on the robot.
    The function returns a list with the color order, as per URFDLB sequence.
    The function also returns a boolean if the HSV analysis has been coherent."""
    
    HSV_analysis=True                                      # flag used to validate the analysis
    
    centers=[4, 13, 22, 31, 40, 49]                        # facelets of the cube's sides centers 
    cube_color_sequence=[4, 13, 22, 31, 40, 49]            # list of center facelets, to later store the cube's side COLORS
    
    if debug:                                              # case debug variable is set True
        print(f'\nHue centers: {Hue[centers[0]]}, {Hue[centers[1]]}, '\
              f'{Hue[centers[2]]}, {Hue[centers[3]]}, {Hue[centers[4]]}, {Hue[centers[5]]}') # feedback is printed to the terminal
    
    VS_centers=[VS_value[facelet] for facelet in centers]  # V-S value measured on the cube side center under analysis
    Hcenters=[Hue[facelet] for facelet in centers]         # Hue value measured on the cube side center under analysis
    
    
    # white and yellow facelet face (center side facelet number)
    white_center=centers[VS_centers.index(max(VS_centers))]   # white center facelet (max V-S distinguishes the white)
    if white_center<27:                       # case the facelet id is < 27  
        yellow_center=white_center+27         # yellow center facelet (yellow is at opposite side of white, in this case requires adding 27)
    else:                                     # case the facelet id is > 27  
        yellow_center=white_center-27         # yellow is at opposite side of white, in this case by subtracting 27
    
    try:
        centers.remove(white_center)          # white facelet is removed from the list of center's facelets
    except:                                   # exception is raised if there are more white centers, in case of a bad color detection
        HSV_analysis=False                    # analysis according to HSV is set False
        if debug:                             # case debug variable is set True
            print('\nIssue with the white_center')   # feedback is printed to the terminal
    
    try:
        centers.remove(yellow_center)         # yellow facelet is removed from the list of center's facelets
    except:                                   # exception is raised if there are more yellow centers, in case of a bad color detection
        HSV_analysis=False                    # analysis according to HSV is set False
        if debug:                             # case debug variable is set True
            print('\nIssue with the yellow_center')    # feedback is printed to the terminal
       
    
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
        if debug:                                   # case debug variable is set True
            print('\nIssue with the red_center')    # feedback is printed to the terminal
    
    try:
        centers.remove(orange_center)         # orange facelet is removed from the list of center's facelets
    except:                                   # exception is raised if there are more orange centers, in case of a bad color detection
        HSV_analysis=False                    # analysis according to HSV is set False
        if debug:                             # case debug variable is set True
            print('\nIssue with the orange_center')   # feedback is printed to the terminal

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
    
    print('Cube_colors_interpr_HSV function has been called')   # feedback is printed to the terminal
    VS_value={}                             # dict to store the V-S (Value-Saturation) value of all facelets
    Hue={}                                  # dict to store the Hue value of all facelets
    
    i=0                                           # iterator index
    for H,S,V in HSV_detected.values():           # Hue, Saturation and Value are retrieved from the HSV dict
        VS_value[i]=int(V)-int(S)                 # V-S (value-Saturation) delta value for all the facelet, populates the related dict
        Hue[i]=int(H)                             # Hue, for all the facelets, populates the related dict
        i+=1                                      # iterator index is increased
    if debug:                                     # case debug variable is set True
        print('\nV-S_values: ', VS_value)         # feedback is printed to the terminal
        print(f'\nHue: ', Hue )                   # feedback is printed to the terminal
    
    # function to get the color (sides) order, and list of the colored center's facelets plus the white center facelet  
    cube_color_sequence, HSV_analysis = retrieve_cube_color_order(VS_value,Hue)    
    
    while HSV_analysis == True:                   # while loop until HSV_analysis is true
        
        # getting the white center's facelet and the colored facelets
        centers=[4, 13, 22, 31, 40, 49]                              # center facelet numbers
        white_center = centers[cube_color_sequence.index('white')]   # facelet number for the white center
        centers.remove(white_center)                                 # white facelet center is removed from the list of facelets centers
        colored_centers=centers                                      # colored centers (without the white)
        
        if debug:                                                    # case debug variable is set True
            
            print('\nWhite facelet number: ', white_center)          # feedback is printed to the terminal
            print('\nCube_color_sequence: ', cube_color_sequence)    # feedback is printed to the terminal
        
        # searching all the 9 white facelets
        Hw,Sw,Vw=HSV_detected[white_center]                              # HSV of the white center
        if debug:                                                        # case debug variable is set True
            print(f'White Hw,Sw,Vw: {Hw}, {Sw}, {Vw}')                   # feedback is printed to the terminal
            print('Colored center facelets numbers: ', colored_centers)  # feedback is printed to the terminal


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

        if debug:                                                       # case debug variable is set True
            print('White facelets: ', white_facelets_list)              # feedback is printed to the terminal
            print('\nHue dict all facelets ', Hue)                      # feedback is printed to the terminal
        
        white_facelets={}  # empty dict to store the association of facelet position for the white facelets
        
        # white facelets are removed from Hue dictionary, as meant for colored facelets
        for facelet in white_facelets_list:                # iteration on all the white facelets listed
            try:
                del Hue[facelet]                           # all white facelets are removed from the Hue dictionary
                white_facelets[facelet]='white'            # dictionary with white facelets is populated
            except:                                        # except is raised if there weren't 6 white facelets
                HSV_analysis = False                       # HSV_analysis is set false
                print('Issue at cube_colors_interpr_HSV function')  # feedback is printed to the terminal
                break                                      # for loop is interrupted
        
        if debug:                                                 # case debug variable is set True
            print('\nHue dict colored facelets ', Hue)            # feedback is printed to the terminal
        
        # facelet's color selection by Hue distance from each center's Hue
        centers=[4,13,22,31,40,49]                                # facelets of centers
        centers.remove(white_center)                              # facelets for colored centers
        cube_color_sequence_no_white=cube_color_sequence.copy()   # cube color sequence copy for list without white
        cube_color_sequence_no_white.remove('white')              # cube color sequence, for colored centers
        
        try:
            Hcenters=[Hue[x] for x in colored_centers]            # list of Hue values for the colored centers
            
        except:                                                   # an exception is raised if the same color is on more centers
            if debug:                                             # case debug variable is set True
                print('\nNot found 6 different colors at center facelets')  # feedback is printed to the terminal
            HSV_analysis=False                                    # HSV_analysis is set false
            break                                                 # for loop is interrupted
        
        if debug:                                                 # case debug variable is set True
            print('\nHcenters (no white): ', Hcenters)            # feedback is printed to the terminal
        
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
            if debug:                                                                      # case debug variable is set True
                print('\nCube_status_detected_colors: ', cube_status_detected)             # feedback is printed to the terminal
                print('\nCube_status_conventional_colors: ', cube_status_URFDLB, '\n')     # feedback is printed to the terminal
            break    # this break quits the while loop (HSV_analysis == True), as the analysis is finally completed 
            
        elif cube_color_sequence == std_cube_color:                                        # case the cube is oriented on the robot as per std URFDLB color
            cube_status_URFDLB = cube_status_detected                                      # detected cube staus is assigned to the URFDLB one
            if debug:                                                                      # case debug variable is set True
                print('\nCube colors and orientation according to URFDLB order')           # feedback is printed to the terminal
                print('\nCube_status_detected_colors: ', cube_status_detected, '\n')       # feedback is printed to the terminal
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

    # Observer= 2nd, Illuminant= D65
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
        After, the robot generates the moves to solve that specific cube status.
        In case of --timer argument, a timer is visualized after the scrambling function."""
    
    global robot_idle  
    
    robot_idle = False              # robot is not anymore idling
    
    disp.show_on_display('CUBE', 'SCRAMBLING', fs1=28, fs2=17)  # feedback is printed on the display
    if not silent:                  # case silent variable is set False
        servo.open_cover()          # top servo is moved to open position
    time.sleep(0.2)                 # little delay, to let user reading the screen
    start_time = time.time()        # current time 
    cc = cubie.CubieCube()          # cube in cubie reppresentation
    cc.randomize()                  # randomized cube in cubie reppresentation 
    random_cube_string = str(cc.to_facelet_cube())   # randomized cube in facelets string reppresentation
    print("Random cube status:", random_cube_string) # feedback is printed to the terminal
    solution, solution_Text = cube_solution(random_cube_string, scrambling = True) # Kociemba solver is called to have the solution string
    print(solution_Text)            # feedback is printed to the terminal
    
    # dict and string with robot movements, and total movements
    _, robot_moves, total_robot_moves = rm.robot_required_moves(solution, solution_Text)
    print('Total robot movements: ', total_robot_moves)  # nice information to print at terminal
    if not robot_stop:              # case there are no request to stop the robot
        robot_move_cube(robot_moves, total_robot_moves, solution_Text, start_time, scrambling=True) # movements to the robot are finally applied
    
    if not robot_stop:              # case there are not request to stop the robot
        if not silent:              # case silent variable is set False
            servo.read()            # top cover positioned to have the PiCamera in read position
        servo.cam_led_test()        # the led on top_cover is shortly activated once the scrambilng is done
    
    
    # eventual timer visualization on display, after the scrambling function
    if args.timer != None:              # case the --timer argument exists
        if args.timer:                  # case the --timer argument has been provided
            
            if args.cycles != None:     # case the --cycles argument exists
                return                  # this function is terminated
            
            if robot_stop:              # case robot is stopped, while scrambling and --timer argument
                disp.clean_display()    # cleans the display
                disp.__init__()         # display is re-initilized (not elegant, yet it removes random issues at robot stop)
                disp.set_backlight(1)   # display backlight is turned on, in case it wasn't
                if not silent:          # case silent variable is set False
                    servo.servo_start_pos(start_pos='read') # servos are placed back to their start position
            

            # cube status inspection time, with countdown time visualization
            inspec_time = 15            # time to let user studying the cube status
            left_time_str ='00:00.0'    # left_time_str initialized to prevent potential missed variable error
            t_ref = dt.datetime.now()   # current datetime is assigned to t_ref as reference
            while not GPIO.input(touch_btn):   # while the button is not pressed
                left_time =  (dt.datetime.now() - t_ref).seconds  # elapsed time in seconds (integer) assigned to d_time
                if left_time  <= inspec_time:  # case left time is smaller than inspect_time
                    left_time_str = str(dt.timedelta(seconds = inspec_time - left_time))[2:]+'.0'  # left time in secs converted to time string
                    disp.show_on_display(left_time_str, 'INSPECT. TIME', fs1=29, fs2=15)  # feedback is printed to the display
                    time.sleep(0.2)     # small delay to limit CPU usage
                else:                   # case left time is bigger than zero
                    if not silent:      # case silent variable is set False
                        servo.open_cover()  # top servo is moved to open position
                    servo.cam_led_test()  # makes a very short led test
                    if not silent:      # case silent variable is set False
                        servo.read()    # top servo is moved to read position
                    break               # while loop is interrupted
            
            
            # cube solving part, with time increment visualization
            d_time_str ='00:00.0'       # d_time_string initialized to prevent potential missed variable error
            disp.show_on_display(d_time_str, 'PRESS TO STOP', fs1=29, fs2=14)   # feedback is printed to the display
            
            if GPIO.input(touch_btn):   # case the button is pressed
                time.sleep(2)           # delay to prevent skipping the next part is button pushed while INSPECT. TIME
            
            timer_timeout = False       # boolean to track whether the timer timeout has been reached
            t_ref = time.time()         # current time assigned to t_ref as reference for overall time
            t_ref2 = time.time()        # current time assigned to t_ref2 as reference for fraction of second
            secs = 0.0                  # variable secs is set to zero
            while not GPIO.input(touch_btn):       # while the button is not pressed
                d_time = int(time.time() - t_ref)  # elapsed time in seconds (float) assigned to d_time
                if time.time() - t_ref2 >= 0.1:    # case other 0.1 secs have elapsed
                    t_ref2 = time.time()           # time reference used for fraction of second is asigned
                    secs = round(secs + 0.1, 1)    # secs is increased by one tent
                    dec = str(round(secs%1 , 2))[1:3] # decimal part of secs is converted to string
                    d_time_str = str(dt.timedelta(seconds=d_time))[2:] + dec   # deltatime in secs converted to time string
                    disp.show_on_display(d_time_str, 'PRESS TO STOP', fs1=29, fs2=14)   # feedback is printed to the display
                    time.sleep(0.03)           # small delay to limit CPU usage
                    if d_time >= 3599:         # case the elapsed time is substantially one hour
                        timer_timeout = True   # timer_timeout variable is set true
                        break                  # while loop is interrupted
            if not timer_timeout:              # case timer_timeout variable is false
                if screen:                     # case there is a screen connected
                    print('Timer stopped at', d_time_str)     # feedback is printed to the terminal
                disp.show_on_display(d_time_str, '', fs1=29)  # feedback is printed to the display
                time.sleep(3)                  # delay to show on screen the elapsed time at button stop
            else:                              # case timer_timeout variable is true
                if screen:                     # case there is a screen connected
                    print('Timer timeout has been reached')   # feedback is printed to the terminal
                disp.show_on_display('TIMEOUT', '', fs1=24)   # feedback is printed to the display
                time.sleep(5)                  # delay to show on screen the TIMEOUT feedback






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
#     print('Moves to cube target',cube_target)
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
    
    folder = pathlib.Path().resolve()                    # active folder (should be home/pi/cubotino/src)  
    folder = os.path.join(folder,'CubesStatusPictures')  # folder to store the collage pictures

    if not os.path.exists(folder):                       # if case the folder does not exist
        os.makedirs(folder)                              # folder is made if it doesn't exist
    fname = folder+'/cube_collage'+timestamp+'.png'      # folder+filename with timestamp for the resume picture
    if debug:                                            # case debug variable is set True
        print('Unfolded cube status image is saved : ', fname) # feedback is printed to the terminal
    status=cv2.imwrite(fname, collage)                   # cube sketch with detected and interpred colors is saved as image
    
    if screen and not robot_stop:                        # case screen variable is set true on __main__
        cv2.namedWindow('cube_collage')                  # create the collage window
        for i in range(5):                               # iteration
            cv2.moveWindow('cube_collage', 0,0)          # move the collage window to (0,0)
            cv2.imshow('cube_collage', collage)          # collage (starting cube status) is shown
            cv2.waitKey(100)                             # showtime of 100ms per iteration
        cv2.waitKey(int(show_time*1000))                 # showtime in secs is trasformed to milliseconds 
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

    if Rpi_ZeroW:                                                  # case Rpi_ZeroW is True (armv6 processor)
        interp_method = cv2.INTER_LINEAR                           # interpolation method is bilinear (prevents openCV from crashing)
    else:                                                          # case Rpi_ZeroW is False (not an armv6 processor)
        interp_method = cv2.INTER_AREA                             # interpolation method resamples using pizel area relation
    collage = cv2.resize(collage, (collage_w, collage_h), interpolation = interp_method) # resized collage
    
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
    
    return facelets
    






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
        edge = int(math.sqrt(area/500))    # use 270 for 3.3%, 500 for 1.3%
    
    for facelet in facelets:                                  # iteration over the 9 facelets just detected
        contour = facelet.get('contour')                      # contour of the facelet under analysis
        candidates.append(contour)                            # new contour is added to the candidates list
#         mask = np.zeros(frame.shape[:2], dtype='uint8')       # mask of zeros is made for the frame shape dimension
#         cv2.drawContours(mask, [contour], -1, 255, -1)        # mask is applied to vsualize one facelet at the time
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
        if debug:                                             # case debug variable is set True
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
    
    facelets = robot_facelets_rotation(facelets) # facelets are rotated to URFDLB related order
    
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
    if debug:                                # case debug variable is set True
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
            if not silent:                       # case silent variable is set False
                servo.read()                     # top cover positioned to have the PiCamera in read position
            servo.cam_led_On(cam_led_bright)     # led on top_cover is switched on 
        
    elif side in (1,2,3):                        # first 4 sides (3 sides apart the one already in front of the camera)
        servo.cam_led_Off()                      # led on top_cover is switched off, for power management
        if not robot_stop and not silent:        # case there are not request to stop the robot nor to silent the servos
            servo.flip()                         # are reached by simply flipping the cube
            servo.cam_led_On(cam_led_bright)     # led on top_cover is switched on 
    
    elif side == 4 :                             # at side 4 is needed to change approach to show side 5 to the camera
        servo.cam_led_Off()                      # led on top_cover is switched off, for power managements
        if not robot_stop and not silent:        # case there are not request to stop the robot nor to silent the servos
            servo.spin_out('CW')                 # one spin (90 deg CW) is applied to the cube
            time.sleep(0.1)
        if not robot_stop and not silent:        # case there are not request to stop the robot nor to silent the servos
            servo.flip()                         # one flip is applied to the cube to bring the side 5 on top
            time.sleep(0.1)
        if not robot_stop and not silent:        # case there are not request to stop the robot nor to silent the servos
            servo.spin_home()                    # spin back home is applied to the cube to show the side 5 to the camera
            time.sleep(0.1)
        if not robot_stop and not silent:        # case there are not request to stop the robot nor to silent the servos
            servo.read()                         # top cover positioned to have the PiCamera in read position
            servo.cam_led_On(cam_led_bright)     # led on top_cover is switched on
            
    elif side == 5 :                             # when side5 is in front of the camera
        servo.cam_led_Off()                      # led on top_cover is switched off, for power management
        for i in range(2):                       # at side 5 are needed two flips to show side 6 to the camera
            if not robot_stop and not silent:    # case there are not request to stop the robot nor to silentthe servos
                servo.flip()                     # cube flipping
        if not robot_stop:                       # case there are not request to stop the robot
            servo.cam_led_On(cam_led_bright)     # led on top_cover is switched on
  
    elif side == 6 :                             # case side equal 6 (it is when the cube has been fully scanned)
        if not robot_stop and not silent:        # case there are not request to stop the robot nor to silent the servos
            servo.open_pos()                     # top_cover is positioned to open position
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
    
    if not silent:                    # case silent variable is set False
        robot_status, robot_time = servo.servo_solve_cube(robot_moves, scrambling, print_out=debug)   # robot solver is called
    else:                             # case silent variable is set True
        robot_status = 'Servos_disabled' # string indicating the robot status
        robot_time = 1000             # robot_time set to 1000, clearly different from robot usual time
        
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
                if not silent:                                 # case silent variable is set False
                    servo.fun(print_out=debug)                 # cube is rotated CW-CCW for few times, as victory FUN 'dance'
            
            fs_row1 = 31 if tot_robot_time >=100 else 35       # font size for display first row
            fs_row2 = 28 if robot_solving_time >=100 else 31   # font size for display second row
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
                    cube_status_string, BGR_mean, HSV_detected, start_time, camera_ready_time, cube_detect_time,
                    cube_solution_time, os_version):
    
    """ Sequence of commands involving the robot, after the cube status detection
    Within this function the robot is called to solve the cube.
    This function calls many other functions, like a picture collage maker to store the detected images of the cube . """
    
    global robot_stop
    
    # dict and string with robot movements, and total movements
    _, robot_moves, total_robot_moves = rm.robot_required_moves(solution, solution_Text) 
#     print(f'\nRobot movements sequence: {robot_moves}')   # nice information to print at terminal, sometime useful to copy 
    
    if solution_Text != 'Error':                # case the solver has returned an error
        print('Total robot movements: ', total_robot_moves)  # nice information to print at terminal, sometime useful to copy

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
                 start_time, camera_ready_time, cube_detect_time, cube_solution_time, robot_solving_time, os_version)
        
        deco_info = (fixWindPos, screen, frame, faces, cube_status, cube_color_sequence,\
                     HSV_analysis, cube_status_string, URFDLB_facelets_BGR_mean, \
                     color_detection_winner, show_time, timestamp) # tuple of variables needed for the decoration function
        decoration(deco_info)  # cals the decoration function, that shows (or just saves, is screen=False) cube's faces pictures  
        
    else:                          # case there is a request to stop the robot
        tot_time_sec = 0           # robot solution time is forced to zero when the solving is interrupted by the stop button
    
    if not silent:                 # case silent variable is set False
        servo.servo_start_pos(start_pos='read')  # servos are placed back to their start position







def robot_timeout_func():
    """ Robot reaction mode in case of timeout.
    Cube state reading, in case of high reflection or pollutted facelets, could get stuck therefore the need for a timeout;
    This function takes care of quitting the reading status."""
    
    servo.cam_led_Off()         # sets off the led at top_cover
    print('\nTimeout for cube status detection: Check if too much reflections, or polluted facelets')
    # feedback is printed to the terminal
    
    disp.show_on_display('READING', 'TIME-OUT', fs1=24)  # feedback is printed to the display
    time.sleep(5)               # time to let possible reading the display
    if not silent:              # case silent variable is set False
        servo.servo_start_pos(start_pos='read')  # top cover and cube lifter to start position

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
        print('\nCube was already solved')                      # feedback is printed to the terminal
        disp.show_on_display('ALREADY', 'SOLVED')               # feedback is printed to the display
        time.sleep(3)                                           # little delay to allows display reading

    return tot_robot_time, robot_solving_time







def check_headers(folder, fname):
    """Checks the existing headers of the log file.
       In case new headers are added, after the first script release, the new headers are added to the log file.
    """
    
    with open(fname, 'r') as f:                     # file is opened, to adjust the headers in case
        line = f.readline().strip('\n').strip('\t')  # file line without spaces nor tab characteres at line start/end
        headers = line.split('\t')                  # file headers are listed
        
        # last header in previous script release
        # 'CubeSolution'                            # latest_header used until 14/01/2023
        # 'Screen'                                  # latest_header used until 26/04/2023
        # 'Flip2close'                              # latest_header used until 10/10/2023
        # 'OS_ver'                                  # latest_header since 10/10/2023
        # 'FCS'                                     # latest header since 20/01/2024
        
        # additional headers since the first script release in a tuple (1st element is the last original header)
        added_headers = ('CubeSolution', 'Screen', 'Flip2close', 'OS_ver', 'FCS')
        latest_header = added_headers[-1]           # header meant to be the latest
        last_header = headers[-1].strip().strip('\t') # header found as last        
        
        if last_header != latest_header:            # case last_header in existing log file differs from latest_header
            print(f"\nOne time action: Adding column header(s) to Cubotino_solver_log.txt as per last script release\n")
            new_headers = ''                        # empty string is assigned to the new_headers variable
            for header in headers:                  # iteration over the current listed headers
                new_headers += header + '\t'        # new_headers string with tab separated headers
            
            # index of the last header used (previous script) in the new headers tupple
            if last_header in added_headers:        # case the latest header in the original .txt file is found in the added_headers tuple
                last_used_header_idx  = added_headers.index(last_header)   # index of the element in tuple is assigned
                
                iterations = len(added_headers)-last_used_header_idx-1
                for i in range(iterations):             # iteration for the missed headers 
                    idx = last_used_header_idx + 1 + i  # pointer for the element in added_header stuple
                    new_headers += added_headers[idx]   # additional header, at idx position in added_headers tupple, is added
                    print("Added header:", added_headers[idx]) # feedback is printed to the terminal
                    if idx < iterations:                # case the iteration has not reached the last loop
                        new_headers += '\t'             # tab separator is added to headers
                    else:                               # case the iteration has reached the last loop
                        new_headers += '\n'             # end of line character is added to headers
                print()

                # writing the updated headers in the 'existing' file, requires a second temporary file
                with open(fname, 'r') as orig:          # Cubotino_solver_log.txt file is temporary opened as orig
                    temp_fname = folder + '/temp.txt'   # name for a temporary file
                    with open(temp_fname, "w") as temp: # temporary file is opened in write mode
                        for i, data_line in enumerate(orig):  # iteration over the lines of data in orig file Cubotino_solver_log.txt
                            if i == 0:                  # case the iteration is on the first row
                                temp.write(new_headers) # the new_headers is written to the temporary file
                            else:                       # case the iteration is not on the first row
                                temp.write(data_line)   # data_line from orig file is written to temp file

                os.remove(fname)                        # fname file (Cubotino_solver_log.txt) is removed
                os.rename(temp_fname, fname)            # temp file is renamed as Cubotino_solver_log.txt (now with latest headers)
            
            else:   # case the latest header in the original .txt file is NOT found in the added_headers tuple
                print("Cannot add the additional headers to the Cubotino_solver_log.txt file")  # feedback is printed to terminal







def log_data(timestamp, facelets_data, cube_status_string, solution, color_detection_winner, \
             tot_robot_time, start_time, camera_ready_time, cube_detect_time, cube_solution_time,\
             robot_solving_time, os_version):
    
    """ Main cube info are logged in a text file
    This function is called obly on the robot (Rpi), to generate a database of info usefull for debug and fun
    BGR color distance is the first approach used to detect cube status, therefore the winner if it succedes.
    HSV color approach is used when the BGR approach fails; If the HSV succedes on cube status detection it become the winner.
    If the cube solver returns an error it means both the approaches have failed on detecting a coherent cube status."""
    
    folder = pathlib.Path().resolve()                   # active folder (should be home/pi/cubotino/src)  
    folder = os.path.join(folder,'CubesDataLog')        # folder to store the relevant cube data
    if not os.path.exists(folder):                      # if case the folder does not exist
        os.makedirs(folder)                             # folder is made if it doesn't exist
    
    fname = folder+'/Cubotino_solver_log.txt'           # folder+filename for the cube data
    if not os.path.exists(fname):                       # if case the file does not exist, file with headers is generated
        if debug:                                       # case debug variable is set True
            print('\nGenerated AF_cube_solver_log_Rpi.txt file with headers') # feedback is printed to the terminal
        
        # headers
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
        m = 'Screen'                                    # 12th column header
        n = 'Flip2close'                                # 13th column header
        o = 'OS_ver'                                    # 14th column header
        p = 'FCS'                                       # 15th column header
        
        # tab separated string of the the headers
        log_data = (a,b,c,d,e,f,g,h,i,k,l,m,n,o,p)      # tuple with the columns headers
        log_data_len = len(log_data)                    # elements in tuple
        s=''                                            # empty string is assigned to the variable s
        for i, header in enumerate(log_data):           # interation trhough the tuple
            s += header                                 # each header is added to the string variable s
            if i <= log_data_len-2:                     # case the iteration has not reached the last tuple element
                s += '\t'                               # tab separator is added to the string variable s
            else:                                       # case the iteration has reached the last tuple element
                s += '\n'                               # end of line character is added to the string variable s
        
        os.umask(0) # The default umask is 0o22 which turns off write permission of group and others
        
        # 'a'means: file will be generated if it does not exist, and data will be appended at the end
        with open(os.open(fname, os.O_CREAT | os.O_WRONLY, 0o777), 'a') as f:    # text file is temporary opened
            f.write(s)               # data is appended
    
    
    else:                                               # case the file does exist
        check_headers(folder, fname)                    # checks if necessary to add new headers to the log file
    
    # info to log
    a=str(timestamp)                                    # date and time
    b=frameless_cube                                    # frameless cube setting
    c=str(color_detection_winner)                       # wich method delivered the coherent cube status
    d=str(round(tot_robot_time,1))                      # total time from camera warmup to cube solved
    e=str(round(camera_ready_time-start_time,1))        # time to get the camera gains stable
    f=str(round(cube_detect_time-camera_ready_time,1))  # time to read the 6 cube faces
    g=str(round(cube_solution_time-cube_detect_time,1)) # time to get the cube solution from the solver
    h=str(round(robot_solving_time,1))                  # time to manoeuvre the cube to solve it
    i=str(facelets_data)                                # according to which methos delivered the solution (BGR, HSV, both)
    k=str(cube_status_string)                           # string with the detected cbe status
    l=str(solution)                                     # solution returned by Kociemba solver
    m='screen'if screen else 'no screen'                # screen presence or absence (it influences the cube detection time)
    n='1' if flip_to_close_one_step else '2'            # flip tp close_cover method used (1 or 2 steps)
    o=str(os_version)                                   # os_version
    p=str(fcs)                                          # fix coordinates system
    
    # tab separated string with info to log
    log_data = (a,b,c,d,e,f,g,h,i,k,l,m,n,o,p)          # tuple with the columns data
    log_data_len = len(log_data)                        # elements in tuple
    s=''                                                # empty string is assigned to the variable s
    for i, data in enumerate(log_data):                 # interation trhough the tuple
        s += data                                       # each data is added to the string variable s
        if i <= log_data_len-2:                         # case the iteration has not reached the last tuple element
            s += '\t'                                   # tab separator is added to the string variable s
        else:                                           # case the iteration has reached the last tuple element
            s += '\n'                                   # end of line character is added to the string variable s

    # 'a' means: file will be generated if it does not exist, and data will be appended at the end
    with open(fname,'a') as f:   # text file is temporary opened
        f.write(s)               # data is appended
        
        if debug:                # case debug variable is set True
            print('\nData is saved in AF_cube_solver_log_Rpi.txt') # feedback is printed to the terminal







def camera_opened_check():
    """ Trivial check if camera is opened, by interrogating it on one of the settings; Funtion returns a boolean."""

    try:
        ret = camera.get_width()        # PiCamera width setting is inquired to the camera
        if ret >= 0:                    # case the returned parameter (width) is bigger than zero
            return True                 # camera is opened, and true is returned
    except:                             # except is raised if camera isn't opened or does not properly reply on the inquiry
        return False                    # camera is evaluated as closed, and false is returned







def close_camera():
    """ Closes the camera object; It's important to close the camera, if the cube detection is performed more than once.
    On PiCamera it's importan to close it, at the end of a cube solving cycle to drop the AWB and Exposure setting used before."""
    
    try:
        camera.close()                  # necessary to close the camera to release the fix settings, like analog/digital gains
        if debug:                       # case debug variable is set True
            print('\nClosed camera')    # feedback is printed to the terminal

    except:
        pass







def robot_set_servo(debug):
    """ The robot uses a couple of servos; This functions positions the servos to the start position."""
    
    # servos are initialized, and set to their starting positions
    return servo.init_servo(debug, start_pos = 'read', f_to_close_mode=flip_to_close_one_step)






def stop_cycle(button):
    """ Function called as an interrupt in case the "start/stop button" is pressed.
    Time delay is used to prevent unwanted interruptions: Button has to be pressed at least for 0.5 seconds
    The function change (global) variables used on robot movements, to prevent further movements to happen
    The function calls the quitting function, that closes the script."""
    
    global robot_stop
    
    time.sleep(0.5)  # delay between function being called, and new GPIO check, to filter unintentionally touches
    
    # case robot not idling and solve or scrambling button is pressed 
    if GPIO.input(touch_btn) and not robot_idle:     # case touch button is 'pressed' while robot not idling
        if not robot_stop:                           # in case the robot was working and not yet stopped
            servo.cam_led_Off()                      # sets off the led at top_cover
            disp.set_backlight(1)                    # display backlight is turned on, in case it wasn't
            if not quit_script:                      # case the quit_script variable is False
                disp.show_on_display('STOPPED', 'CYCLE') # feedback is printed to the display
            robot_stop = True                        # global flag to immediatly interrup the robot movements is set
            if not silent:                           # case silent variable is set False
                servo.stopping_servos(print_out=debug)   # function to stop the servos    
            time.sleep(1)
            if not quit_script:                      # case the quit_script variable is False
                disp.show_on_display('STOPPED', 'CYCLE') # feedback is printed to the display a second time
            quit_func(quit_script=False)             # quit function is called, without forcing the script quitting
    
    elif GPIO.input(touch_btn) and robot_idle:       # case touch button is 'pressed' while robot is idling
        servo.cam_led_Off()                          # sets off the led at top_cover
        disp.set_backlight(1)                        # display backlight is turned on, in case it wasn't
        if not quit_script:                          # case the quit_script variable is False
            disp.show_on_display('QUITTING', '')     # feedback is printed to the display
        robot_stop = True                            # global flag to immediatly interrup the robot movements is set
        if not silent:                               # case silent variable is set False
            servo.stopping_servos(print_out=debug)   # function to stop the servos    
        time.sleep(1)
        if not quit_script:                          # case the quit_script variable is False
            disp.show_on_display('QUITTING', '')     # feedback is printed to the display a second time
        quit_func(quit_script=False)                 # quit function is called, without forcing the script quitting
            

        





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







def cpu_temp(side, delay=5):
    """ Funtion to read/print CPU temperature at Raspberry pi.
    This gives an idea about the temperature into the robot case."""
    
    try:
        tFile = open('/sys/class/thermal/thermal_zone0/temp')      # file with the cpu temp, in mDegCelsius (text format)
        cpu_temp = round(float(tFile.read()) /1000, 1)             # tempertaure is converted to (float) degCelsius

    except:            # case of errors
        tFile.close()  # file is closed
    
    if side!=0:                                                    # case the cube side is not zero
        if not fahrenheit:                                         # case farenheiy is set False in __main__
            print(f'\nCPU temperature: {cpu_temp} degrees C')      # feedback is printed to the terminal
            disp.show_on_display('CPU TEMP.', str(str(cpu_temp)+' degC'), fs1=20, fs2=20)    # feedback is printed to the display
        else:                                                      # case farenheiy is set True in __main_
            cpu_temp = round((cpu_temp * 9 / 5) + 32 , 1)          # temperature conversion to fahrenheit
            print(f'\nCPU temperature: {cpu_temp} fahrenheit deg') # feedback is printed to the terminal
            disp.show_on_display('CPU TEMP.', str(str(cpu_temp)+' degF'), fs1=20, fs2=20)    # feedback is printed to the display
                
    import time        # time module is imported
    if screen:         # case screen variable is set True
        delay = 1      # delay variable is set to 1
    time.sleep(delay)  # sleep time is applied as screen reading time








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
        print('Internet is connected')                  # feedback is printed to the terminal
        internet = True                                 # internet variable is set true
    except:                                             # exception is used as no internet availability
        print('No internet connection')                 # feedback is printed to the terminal
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
                    print('Time system is synchronized: ', str(date_time))        # feedback is printed to the terminal
                    disp.show_on_display('TIME SYSTEM','UPDATED', fs1=16)   # feedback is printed to the display
                    time.sleep(1.5)                              # sleep time to let the user reading the display
                    disp.show_on_display(str(date_time[11:]), '', fs1=24)   # feedback is printed to the display
                    break                                        # while loop is interrupted
                else:                                            # case the timedatectl status returns false
                    if once:                                     # case the variable once is true
                        print('Waiting for time system update')  # feedback is printed to the terminal
                        once = False                             # variable once is set false, to print a feedback only once
                    time.sleep(0.5)                              # sleep time before inquiry to timedatectl status again
                    i+=1                                         # iterator is increased
            except:                                              # case there is an exception
                break                                            # while loop is interrupted
                
    else:                                                        # case the is not an internet connection
        print('Time system not synchronized yet')                # feedback is printed to the terminal
        disp.show_on_display('TIME SYSTEM','NOT UPDATED', fs1=16, fs2=15)   # feedback is printed to the display
        time.sleep(1.5)                                          # sleep time to let the user reading the display






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
    print('#############################################################################\n')
    
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
        cpu_temp(side, delay=2) # cpu temperature is verified, printed at terminal and show at display for delay time
        print(f'\n#######################    END  SOLVING CYCLE  {solv_cycle}   ##########################')
    







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
        show_cube(date=True, cycle=cycle, total=total) # show_cube function is called
        robot_idle = False                # robot idle set off to allows cycle stopping while pause
        disp.set_backlight(1)             # display backlight is turned on, in case it wasn't
        start = time.time()               # time reference
        screen1=True                      # boolean used to alternate two prints at the screen
        
        print()
        elapsed_t = 0                     # zero is assigned to the variable counting the elapsed time with even integers
        while time.time()-start < cycle_pause: # case the elapsed time is smaller than cycle_pause time
            time_left = int(cycle_pause-(time.time()-start))  # time_left is calculated and converted to integer
            
            if robot_stop:                # case of request to stop the robot
                stop_or_quit()            # stop or quit function is called
                break                     # while loop is interrupted
            
            else:                         # case no request to stop the robot
                if time_left % 2 == 0:    # case the time_left is even
                    elapsed_t+=2          # elapsed time is increased by two
                    progress = round(elapsed_t/(cycle_pause),3)   # progress 
                    print("\rNext cycle:   elapsed time {0:}s   [{1:50s}] {2:}%   still left {3:}s          ".format(
                        elapsed_t, '.' *int(progress*50), round(progress*100,1), time_left), flush=True, end="")
                    
                    if screen1:           # case boolean screen1 is true
                        disp.show_on_display('FOR CYCLE', f'{cycle+1} / {total}', fs1=21, fs2=24)  #feedbak is print to to the display
                        screen1 = False   # boolean screen1 is set false
                    else:                 # case boolean screen1 is false
                        disp.show_on_display('WAIT TIME', f'{time_left} s', fs1=21, fs2=24)  #feedbak is print to to the display
                        screen1 = True    # boolean screen1 is set true
                    
                if elapsed_t >= cycle_pause:   # case the time to wait is over
                    time.sleep(2)         # little time to let visible on screen/terminal that the timer is over
                    print("\r{}\n".format(' '*130))   # prints 100 empty characters to overwrite the progress bar at the terminal
                    try:                  # tentative
                        cv2.destroyAllWindows()   # closes al the graphical windows
                    except:               # case an exception is raised
                        pass              # no action
                    break                 # the while loop is interrupted     
                
                else:                     # case there still is time to wait
                    time.sleep(1)         # system can sleep for 1 s





def show_cube(date=None, cycle=None, total=None):
    """function that activates the Top_cover led and the camera to retrieve one frame.
        The frame is plot on a window, by adding datetime as reference."""
    
    global camera, width, height
     
    if screen:                                # case screen variable is set true on __main__
        if camera == None:                    # case the camera object does not exist
            camera, width, height = set_camera()  # camera object is created
        
        servo.cam_led_On(cam_led_bright)      # led on top_cover is switched on
        time.sleep(1)                         # little delay to let the led and camera to stabilize
        frame, w, h = read_camera()           # camera start reading the cube, and adjusts the awb/exposure
        servo.cam_led_Off()                   # sets off the led at top_cover
        
        if date!=None:                        # case variable date is not None
            h, w = frame.shape[:2]            # frame height and width
            bg = np.zeros([40, w, 3],dtype=np.uint8)    # empty array having same width of frame images
            bg.fill(230)                      # array is filled with light gray
            datetime = dt.datetime.now().strftime('%Y/%m/%d  %H:%M:%S')  # date_time variable is assigned
            cv2.putText(bg, datetime, (20, 28), font, .7, (0,0,0), lineType)  # add text to background
            frame=np.vstack([bg, frame])      # bg array (background with text) is vertically stack to frame 
        
        if cycle!=None and total!=None:       # case variables cycles and total are not None
            h, w = frame.shape[:2]            # frame height and width
            bg = np.zeros([40, w, 3],dtype=np.uint8)   # empty array having same dimensions of cube's faces images
            bg.fill(230)                      # array is filled with light gray
            text = f'Cycle {cycle} / {total}' # text to be printed
            cv2.putText(bg, text, (20, 28), font, 0.7, (0,0,0), lineType)     # add text to background
            frame=np.vstack([bg, frame])      # bg array (background with text) is vertically stack to frame 
        
        if fixWindPos:                        # case the fixWindPos variable is chosen
            cv2.namedWindow('cube')           # create the cube window
            cv2.moveWindow('cube', 0,0)       # move the window to (0,0)
        cv2.imshow('cube', frame)             # shows the frame 
        cv2.waitKey(2000)                     # refresh time

    







def robot_reset():
    """reset the servo to start position, and reset the display."""
    
    global robot_stop
    
    disp.set_backlight(1)                  # display backlight is turned on, in case it wasn't
    disp.show_on_display('ROBOT', 'RESTARTING', fs1=30, fs2=18) # feedback is printed to display
    time.sleep(2)                          # delay time to let user reading the display
    if not silent:                         # case silent variable is set False
        servo.stop_release(print_out=debug)    # stop_release at servo script, to enable the servos movements
        servo.servo_start_pos(start_pos='read')   # servos are rotated to the start position
    robot_stop = False                     # flag used to stop or allow robot movements
    if not GPIO.input(touch_btn):          # case the button has been released
        disp.clean_display()               # cleans the display
        disp.__init__() # display is re-initilized (not elegant, yet it removes random issues at robot stop)
    






def stop_or_quit():
    """Distinguishes the request to just stop a cycle (scrambling or solving) or to SHUT the Rpi OFF.
        The input is the solve button or scrambling button, related to pressing time.
        When the button is pressed longer than warning_time, a warning message is displayed.
        When the button is not released within the quit_time, the Rpi SHUT-OFF."""
    
    print('Stop request')                    # feedback is printed to the terminal
    ref_time=time.time()                     # reference time used to measure how long the button has been kept pressed
#     warn_time = 1.5         #(AF 1.5)        # delay used as threshold to print a warning on display
#     quit_time = 4.5         #(AF 4.5)        # delay used as threshold to quit the script
    warning = False                          # warning is set false, to warn user to keep or release the button
    quitting = False                         # quitting variable is set false

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
                if Rpi_ZeroW:                             # case Rpi_ZeroW is True (armv6 processor)
                    if time.time() - ref_time >= quit_time - 0.5:   # case time elapsed is >= quit time reference
                        quitting = True                   # quitting variable is set true
                else:                                     # case Rpi_ZeroW is False (not an armv6 processor)
                    if time.time() - ref_time >= quit_time:   # case time elapsed is >= quit time reference
                        quitting = True                   # quitting variable is set true
                if quitting:                              # case the quitting variable is true
                    print('Quitting request')             # feedback is printed to display
                    for i in range(5):                    # iteration for  5 times
                        disp.show_on_display('SHUTTING', 'OFF', fs1=20, fs2=28) # feedback is printed to display
                    if not silent:                        # case silent variable is set False
                        servo.stop_release(print_out=debug)   # stop_release at servo script, to enable the servos movements
                        servo.init_servo(print_out=debug)     # servos are moved to their starting positions
                    quit_func(quit_script=True)           # qutting function is called, with script clossure
    
    time.sleep(5)       # time to mask the button pressing time (interrupt), eventually used to quit the script
    robot_reset()       # call the robot reset function






def quit_func(quit_script, error = False):   
    """ Quitting function, that properly closes stuff: Camera is closed, OpenCV windows are closed,
        Servos are positioned to their start position, Led at top_cover is switched off, set GPIO used for PWM to low level."""

    global quitting
    
    if error:                      # case an error has been raised by the script
        quit_script = True         # quit_script is set true
        
    if not quit_script:            # case the quit_script variable is false (tipically every time this function is called)
        try:
            if not idling:
                disp.set_backlight(1)   # display backlight is turned on, in case it wasn't
                disp.show_on_display('STOPPED', 'CYCLE')   # feedback is printed to the display
            if not quitting:            # case quitting variable is false, meaning all the times entering the quit_func
                quitting = True         # (global) quitting variable is set true
        except:
            pass
        
        try:
            cv2.destroyAllWindows()     # closes al the graphical windows
        except:
            print("Raised exception while cv2.destroyAllWindows at script quitting") # feedback is printed to the terminal
            pass
       
        try:
            servo.cam_led_Off()         # sets off the led at top_cover 
        except:
            print("Raised exception while servo.cam_led_Off at script quitting")    # feedback is printed to the terminal
            pass

    
    
    ##################### closing the python script, and via bash shutting off the raspberry OS ########
    elif quit_script:                             # case the quit_script is set true (long button pressing)
        print('Script quitting request')          # feedback is printed to the terminal
        
        if not (error or picamera_test):          # case no errors were raised by the script
            try:                                  # tentative
                disp.set_backlight(1)             # display backlight is turned on, in case it wasn't
                disp.show_on_display('SHUTTING', 'OFF', fs1=20, fs2=28) # feedback is printed to the display
                time.sleep(1.5)                   # wait time to let the message visible on the display
                
                countdown = 3                     # count-down variable
                if cover_self_close:              # case the cover_self_close is set True at the settings file
                    disp.show_on_display('CLOSING', 'COVER',fs1=24, fs2=26) # feedback is printed to the display
                else:                             # case the cover_self_close is not set True at the settings file
                    disp.show_on_display('SHUTTING', 'OFF',fs1=20, fs2=28) # feedback is printed to the display
                time.sleep(1.5)                   # wait time to let the message visible on the display
                for i in range(countdown,-1, -1): # iteration down the countdown variable
                    dots = ''                     # dot string variable is set empty
                    for k in range(min(i,3)):     # iteration over the residual cont-down, with max of three
                        dots = dots + '.'         # dot string variable adds a dot character          
                    row2_text = str(i) + dots     # string variable to be printed on the second disply row
                    if cover_self_close:          # case the cover_self_close is set True at the settings file
                        # feedback is printed to the display
                        disp.show_on_display('CLOSING IN', row2_text, x1=20, y1=20, x2=20, y2=50, fs1=18, fs2=60)
                    else:                         # case the cover_self_close is not set True at the settings file
                        # feedback is printed to the display
                        disp.show_on_display('OFF IN', row2_text, x1=20, y1=20, x2=20, y2=50, fs1=18, fs2=60)
                    if i > 0:                     # case the cont-down is above 0
                        time.sleep(1)             # wait time to let the message visible on the display
                if cover_self_close:              # case the cover_self_close is set True at the settings file
                    if not silent:                # case silent variable is set False
                        servo.close_cover()       # top cover is closed
                time.sleep(0.5)                   # time to allow the servo reaching the close position
                disp.set_backlight(0)             # display backlight is turned off
            except:
                pass
        
        try:
#             servo.servo_off()         # PWM is stopped at the servos GPIO pins
#             time.sleep(0.1)           # little delay
            servo.quit_func()         # sets to low the GPIO pins used as output
        except:
            print("Raised exception while servo.quit_func at script quitting")   # feedback is printed to the terminal
            pass
         
        try:
            cv2.destroyAllWindows()   # closes al the graphical windows
        except:
            print("Raised exception while cv2.destroyAllWindows at script quitting")   # feedback is printed to the terminal
            pass
       
        try:
            servo.cam_led_Off()       # sets off the led at top_cover 
        except:
            print("Raised exception while servo.cam_led_Off at script quitting")   # feedback is printed to the terminal
            pass
        
        try:
            close_camera()            # closes the camnera object (should be the latest command, as per close camera)
        except:
            print('Issues at camera closing while quitting')    # feedback is printed to the terminal
            pass
        
        try:
            disp.clean_display()      # cleans the display
        except:
            print("Raised exception while clean_display at script quitting")   # feedback is printed to the terminal
            pass
        
        try:
            disp.set_backlight(0)     # display backlight is turned off
        except:
            pass
        
        try:
            GPIO.setup(4, GPIO.OUT, initial=GPIO.LOW) # GPIO 4 (display backlight) is set as output, and forced low
        except:
            print("Raised exception while setting low the display backlight GPIO at script quitting")   # feedback is printed to the terminal
        
        
        # below 'if' block is to kill the bash process that has eventually been launched with this script.
        # this approach helps during development, to stop the script without starting the Rpi shutt off process
        try: 
            if GPIO.input(touch_btn):                   # case button is pressed
                print("Request to quit the script without shutthing off the Raspberri Pi")
                disp.set_backlight(1)                   # display backlight is turned on for the first time
                disp.show_on_display('EXITING', 'SCRIPT', fs1=24, fs2=26) # feedback is printed to the display
                time.sleep(1)                           # some little delay
                process_to_kill = "Cubotino_T_bash.sh | grep -v grep | grep -v Cubotino_T_terminal.log"  # string to find the process PID to kill
                nikname = "Cubotino_T_bash.sh"          # process name
                kill_process(process_to_kill, nikname)  # call to the killing function
                time.sleep(1)                           # some little delay
                disp.show_on_display('SCRIPT', 'ENDED', fs1=24, fs2=26) # feedback is printed to the display
                time.sleep(2)                           # some little delay
                disp.set_backlight(0)                   # display backlight is turned off
                sys.exit(2)                             # script is quitted with error 2 (right after killing the bash script)
        except:
            pass
        
        if error:                     # case an error has been raised by the script
            sys.exit(1)               # script is quitted with error
        else:                         # case no errors were raised by the script
            sys.exit(0)               # script is quitted without error
    
    else:
        print('Something wrong at def quit_func')   # feedback is printed to the terminal
        raise # Raising Exception







def plot_to_display(side, BGR_mean=[]):
    
    if len(BGR_mean) == 0:                       # case no brg data
        disp.show_face(side)                     # side is sent to display the (empty) cube face frame.
    
    else:                                        # case of brg data
        fclt_start = (0, 0, 45, 27, 18, 9, 36)   # first facelet per each side (side 0 is a dummy)
        bgr_side = BGR_mean[fclt_start[side]:fclt_start[side]+9] # BGR of the detected side
        bgr_rotated = []                         # list for detected bgr rotated as per user in front of the display
        
        if side in(1,3,4,5):                     # case side requires facelets order to rotate 90deg CW
            fclt_order = (6,3,0,7,4,1,8,5,2)     # facelet order
        elif side in(2,6):                       # case side requires facelets order to rotate 90deg CCW
            fclt_order = (2,5,8,1,4,7,0,3,6)     # facelet order
        else:                                    # case side is not include in 1 to 6
            print("Error at plot_to_display func")   # feedback is printed to the terminal
        
        for i in range(9):                       # iteration over the 9 facelets
            bgr = bgr_side[fclt_order[i]]        # bgr components the detected facelet i
            bgr_rotated.append((bgr))            # bgr is appended to the list of rotated bgr facelets
        
        disp.show_face(side,bgr_rotated)         # side and bgr for the 9 facelets are sent to display.
    







def kill_process(process, nikname):
    """function to kill the process in argument."""
    
    import os
    import psutil
    import subprocess
    
    p_name = "ps ax | grep -i " + process            # process name to search the PID
    try:                                             # tentative
        for line in os.popen(p_name):                # iterating through each instance of the process
            fields = line.split()                    # output from px aux
            pid = fields[0]                          # extracting Process ID from the output
            cmd = "sudo kill -9 %s" % pid            # command to terminate the process ( -9 to kill force fully)
            result = subprocess.run(cmd, shell=True) # executing the command to terminate the process, and collecting the output
            if result.returncode == 0:               # case the returncode equals to zero (command executed with success)
                print(f"\nProcess {nikname} is terminated")  # feedback is printed to the terminal
                
    except:                                          # case an exception is raise
        print("Error at the kill_process function")  # feedback is printed to the terminal
        raise                                        # error is raised






def test_picamera():
    """funtion to allow a quick feedback of PiCamera working fine, when the robot is not fully assembled yet."""
    
    global np, time, cv2
    global side, robot_stop, cycles_num, camera, width, height
    
    # import libraries
    import numpy as np                              # data array management
    import time                                     # time package
    import cv2                                      # computer vision package                      
    
    side = 0                                        # zero is assigned to side (variable used by webcam() function)
    cycles_num = 0                                  # zero is assigned to cycles_num (variable used by webcam() function)
    robot_stop = False                              # false is assigned to robot_stop (variable used by webcam() function)
    
    camera, width, height = set_camera()            # camera relevant info are returned after cropping, resizing, etc
    frame, w, h = read_camera()                     # video stream and frame dimensions
    font, fontScale, fontColor, lineType = text_font()  # font characteristics
    font_k = w/640                                  # font size adjustment factor, based on frame width
    
    show_time = 60
    start = time.time()                             # current time is assigned to variable start
    fps_start = start                               # initial time reference for FPS calculation
    fps = 1                                         # initial value to feed the FPS low pass filter of
    while time.time()-start < show_time:            # iteration until the elapsed time is smaller than show_time
        countdown = int(show_time-(time.time()-start))  # remaining time 
        frame, w, h = read_camera()                 # video stream and frame dimensions
        background_h=42                             # height of a black bandwidth used as back ground for text
        cv2.rectangle(frame, (0, 0), (w, background_h), (0,0,0), -1)    # black background bandwidth at frame top
        cv2.rectangle(frame, (0, h-background_h), (w, h), (0,0,0), -1)  # black background bandwidth at frame bottom
        cv2.putText(frame, f'PiCamera TEST in CUBOTino   (FPS: {fps})', (10, 30), font, fontScale*font_k, fontColor, lineType)
        text = f'Closing in {countdown} seconds,   ESC to quit earlier'
        cv2.putText(frame, text, (10, int(h-12)), font, fontScale*font_k, fontColor, lineType)
        cv2.imshow('PiCamera test', frame)          # shows the frame 
        key = cv2.waitKey(1)                        # refresh time is minimized (1ms), yet real time is much higher
        if key == 27 & 0xFF:                        # ESC button method to close CV2 windows
            break                                   # while loop is interrupted
        
        fps_end = time.time()                       # second time reference for FPS calculation
        fps = round(0.92*fps + 0.08*(1/(fps_end-fps_start)),1)  # FPS calculation with low-pass filter
        fps_start = fps_end                         # new initial time reference for FPS calculation
        
    
    try:                                            # tentative
        close_camera()                              # close the camera object
    except:                                         # case of exception
        pass                                        # do nothing
    
    try:                                            # tentative
        cv2.destroyAllWindows()                     # all open windows are closed
    except:                                         # case of exception
        pass                                        # do nothing
    
    quit_func(quit_script=True)                     # quitting funtion is used to close all the threads and the script







def tune_image_setup_UpTo20240109(display, gui_debug):
    "function used by the GUI script for the tuning"
    
    global np, time, sys, cv2
    global disp, debug, screen, robot_stop, picamera_test, cv_wow, Rpi_ZeroW, side, cycles_num
    global camera
    global width, height
    
    # import libraries
    import numpy as np                              # data array management
    import time                                     # time package
    import sys                                      # Python Runtime Environment livrary
    import cv2                                      # computer vision package
    
    disp = display                                  # display object, defined on the GUI sript
    debug = gui_debug                               # debug variable set in the GUI script
    screen = True                                   # scrren is always true when using the GUI
    robot_stop = False                              # false is assigned to robot_stop (variable used by set_camera() function)
    picamera_test = True                            # this variable helps to use limited functionality from this script
    cv_wow = False                                  # cw_wow is set false
    Rpi_ZeroW = True                                # Rpi_ZeroW is set true for larger compatibility
    import_parameters(debug)                        # imports the robot parameters (not the servo ones)
    side = 0                                        # zero is assigned to side (variable used by set_camera() function)
    cycles_num = 0                                  # zero is assigned to cycles_num (variable used by set_camera() function)
    camera, width, height = set_camera()            # camera is defined
    
    return cv2, camera, width, height


def tune_image_setup(expo_shift, display, gui_debug):
    "function used by the GUI script for the tuning"
    
    global np, time, sys, cv2
    global disp, debug, screen, robot_stop, picamera_test, cv_wow, Rpi_ZeroW, side, cycles_num
    global camera, width, height
    
    # import libraries
    import numpy as np                              # data array management
    import time                                     # time package
    import sys                                      # Python Runtime Environment livrary
    import cv2                                      # computer vision package
    
    disp = display                                  # display object, defined on the GUI sript
    debug = gui_debug                               # debug variable set in the GUI script
    
    screen = True                                   # scrren is always true when using the GUI
    robot_stop = False                              # false is assigned to robot_stop (variable used by set_camera() function)
    picamera_test = True                            # this variable helps to use limited functionality from this script
    cv_wow = False                                  # cw_wow is set false
    Rpi_ZeroW = True                                # Rpi_ZeroW is set true for larger compatibility
    import_parameters(debug)                        # imports the robot parameters (not the servo ones)
    side = 0                                        # zero is assigned to side (variable used by set_camera() function)
    cycles_num = 0                                  # zero is assigned to cycles_num (variable used by set_camera() function)
    camera, width, height = set_camera(expo_shift)  # camera is defined
    
    return cv2, camera, width, height








def start_up(first_cycle=False, set_cropping=False):
    """ Start up function, that aims to run (once) all the initial settings needed."""
    
    # global variables
    global camera, width, height, w, h                    # camera and frame related variables
    global show_time, cam_led_bright                      # camera and frame related variables
    global sides, side, faces, prev_side, BGR_mean, H_mean, URFDLB_facelets_BGR_mean   # cube status detection related variables
    global timeout, detect_timeout, robot_stop                             # robot related variables
    global font, fontScale, fontColor, lineType                            # cv2 text related variables
    global f_coordinates



    # series of variables settings, to re-set at each cycle
    prev_side=0                      # set the initial previous side to zero
    BGR_mean=[]                      # empty list to be filled with with 54 facelets BGR colors while reading cube status
    H_mean=[]                        # empty_ list to be filled with with 54 facelets HUE values, while reading cube status
    URFDLB_facelets_BGR_mean=[]      # empty list to be filled with with 54 facelets colors, ordered according URFDLB order
    faces={}                         # dictionary that store the image of each face
    side=0                           # set the initial cube side (cube sides are 1 to 6, while zero is used as starting for other setting)
    cpu_temp(side)                   # cpu temp is checked at cube solving-end
    robot_stop = False               # flag to stop the robot movements
    timeout = False                  # timeout flag is initialli set on False
    
    # series actions, or variables setting, to be done only at the first cycle
    if first_cycle and not set_cropping:
        cpu_temp(side=10, delay=3)   # cpu temp is checked at start-up
        time_system_synchr()  # checks the time system status (if internet connected, it waits till synchronization)
        
#         cam_led_bright = 0.1           #(AF 0.1)           # set the brighteness on the led at top_cover (admitted 0 to 0.3)
#         detect_timeout = 40             #(AF 40)            # timeout for the cube status detection (in secs)
#         show_time = 7                      #(AF 7)             # showing time of the unfolded cube images (cube initial status)
        
        if screen:                                             # case there is a screen connected
            detect_timeout = int(1.5 * detect_timeout)         # cube status detection timeout is increased
        if cv_wow:                                             # case the cv image analysis plot is set true
            detect_timeout = int(3 * detect_timeout)           # cube status detection timeout is increased
        sides={0:'Empty',1:'U',2:'B',3:'D',4:'F',5:'R',6:'L'}  # cube side order used by the robot while detecting facelets colors
        font, fontScale, fontColor, lineType = text_font()     # setting text font paramenters
        camera, width, height = set_camera()                   # camera object is created
        robot_set_GPIO()                                       # GPIO settings used on the Raspberry pi
        robot_init_status = robot_set_servo(debug)             # settings for the servos
        if not robot_init_status:                              # case the servo init function returns False
            print("Error occurs at servos init")               # feedback is printed to the terminal
            disp.set_backlight(1)                              # display backlight is turned on, in case it wasn't
            disp.show_on_display('SERVOS', 'ERROR', fs1=26, fs2=26) # feedback is printed to display
            time.sleep(5)
            disp.show_on_display('SHUTTING', 'OFF', fs1=20, fs2=28) # feedback is printed to display
            time.sleep(5)
            quit_func(quit_script=True)                        # qutting function is called, with script clossure
            
        f_coordinates = load_coordinates()








def cubeAF():
    """ This function is substantially the main function, covering all the different phases after the initial settings:
        Camera setting for 1st side and remaining
        Keeps interrogating the camera
        Cube status detection
        Cube solver call
        Cube solving at robot."""
    
    # global variables
    global os_version, camera, width, height, h, w, cam_led_bright, fixWindPos, screen    # camera and frame related variables          
    global sides, side, prev_side, faces, BGR_mean, H_mean, URFDLB_facelets_BGR_mean      # cube status detection related variables
    global font, fontScale, fontColor, lineType                                           # cv2 text related variables
    global servo, robot_stop, robot_idle, timeout, detect_timeout                         # robot related variables
    global fcs


    robot_idle = False                              # robot is not anymore idling
    side = 0                                        # side zero is used for some initialization processes
    
    if not camera_opened_check():                   # checks if camera is responsive
        print('\nCannot open camera')               # feedback is printed to the terminal
        disp.show_on_display('CAMERA', 'ERROR', fs1=26, fs2=26)     # feedback is printed to the display
        time.sleep(10)                              # delay to allows display to be read
        quit_func(quit_script=True)                 # script is closed, in case of irresponsive camera
    
    start_time = time.time()                        # initial time is stored before picamera warmup and setting
    faces.clear()                                   # empties the dict of images (6 sides) recorded during previous solving cycle
    facelets = []                                   # empties the list of contours having cube's square characteristics
    all_coordinates = []                            # empties the list of contours centers coordinate as reference for next facelet search
    robot_to_cube_side(side, cam_led_bright)        # robot set with camera on read position
    servo.cam_led_On(cam_led_bright)                # led on top_cover is switched on before the PiCamera warmup phase     
    
    if not robot_stop:                              # case there are no requests to stop the robot
        robot_consistent_camera_images(debug, os_version, camera, start_time)  # sets PiCamera to capture consistent images
        timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S') # date_time variable is assigned, for file name and log purpose
        camera_ready_time=time.time()               # time stored after picamera warmup and settings for consistent pictures
        side = 1                                    # side is changed to 1, as the cube faces are numbered from 1 to 6
        fcs = 0                                     # fcs = fix coordinates system, is initially set False (0)
        t_ref = time.time()                         # timer is reset (timer used on each face detection to eventually witch to fix coordinates)
    
    
    while not robot_stop:                           # substantially the main loop, it can be interrupted by quit_func() 
        if robot_stop:                              # case the robot has been stopped
            break                                   # while loop is interrupted
        
        plot_to_display(side)                       # feedback is printed to the display
        frame, w, h = read_camera()                 # video stream and frame dimensions
        
        if screen:                                  # case screen variable is set true on __main__
            cv2.namedWindow('cube')                 # create the cube window
            if fixWindPos:                          # case the fixWindPos variable is chosen  
                cv2.moveWindow('cube', 0,0)         # move the window to (0,0)
        
        if not robot_stop:                                   # case there are no requests to stop the robot
            (contours, hierarchy)=read_facelets(frame, w, h) # reads cube's facelets and returns the contours
            candidates = []                                  # empties the list of potential contours
        
        if not robot_stop and hierarchy is not None:         # analyze the contours in case these are previously retrieved
            hierarchy = hierarchy[0]                         # only top level contours (no childs)
            facelets = []                                    # empties the list of contours having cube's square characteristics
            
            if timeout or robot_stop:                        # in case of reached timeout or stop_button pressed
                quit_func(quit_script=False)                 # quit function is called, withou forcing the script quitting
                break                                        # while loop is interrupted
            
            for component in zip(contours, hierarchy):       # each contour is analyzed   
                contour, hierarchy, corners = get_approx_contours(component)  # contours are approximated
                
                if  time.time() - camera_ready_time > detect_timeout:  # timeout is calculated for the robot during cube status reading
                    timeout = robot_timeout_func()                     # in case the timeout is reached
                    break                                              # for loop is interrupted
                
                if robot_stop:                                         # case the robot has been stopped
                    break                                              # for loop is interrupted
                
                if screen and not robot_stop:                          # case screen variable is set true on __main__
                    cv2.imshow('cube', frame)                          # shows the frame 
                    cv2.waitKey(1)      # refresh time is minimized to 1ms, refresh time mostly depending to all other functions
                
                if corners==4:                                         # contours with 4 corners are of interest
                    facelets, frame, fix_c = get_facelets(facelets, frame, contour, hierarchy, t_ref) # returns a dict with cube compatible contours
                    fcs += fix_c                                       # fcs (Fix Coordinates System) is incremented
                    
                if len(facelets)==9:                                   # case there are 9 contours having facelets compatible characteristics
                    facelets = order_9points(facelets, new_center=[])  # contours are ordered from top left
                    d_to_exclude = distance_deviation(facelets)        # facelets to remove due inter-distance not as regular 3x3 array
                    if len(d_to_exclude)>=1:                           # check if any contour is too far to be part of the cube
                        d_to_exclude.sort(reverse=True)                # reverse the contours order
                        for i in d_to_exclude:                         # remove the contours too faar far to be part of the cube
                            facelets.pop(i)                            # facelet is removed
                
                if len(facelets)==9:                                   # case having 9 contours compatible to a cube face
                    if fcs == 0:                                       # case facelets were detected without the fix coordinates system method
                        coordinates=[]                                 # empty list to store the facelets coordinates of the last scanned face
                        for i in range(9):                             # iteration over the 9 facelets
                            coordinates.append(facelets[i]['cx'])      # x coordinate is retrieved and appended to the coordinates list
                            coordinates.append(facelets[i]['cy'])      # y coordinate is retrieved and appended to the coordinates list
                        all_coordinates.append(coordinates)            # 9 facelets centers coordinates are appended to all_coordinates (all faces)
                    
                    facelets = robot_facelets_rotation(facelets)       # order facelets as per viewer POW (due to cube/camera rotations on robot)
                    read_color(frame, facelets, candidates, BGR_mean, H_mean)   # each facelet is read for color
                    URFDLB_facelets_BGR_mean = URFDLB_facelets_order(BGR_mean)  # facelets are ordered as per URFDLB order
                    plot_to_display(side, URFDLB_facelets_BGR_mean)    # detected colour are plot to the display
                    faces = face_image(frame, facelets, side, faces)   # image of the cube side is taken for later reference
                    
                    if screen and not robot_stop:                # case screen variable is set true on __main__
                        if cv_wow:                               # case the cv image analysis plot is set true                              
                            cv2.destroyWindow('cube')            # cube window is closed
                            show_cv_wow(frame, time = 4000 if Rpi_ZeroW else 2000)  # call the function that shows the cv_wow image
                        else:                                    # case the cv image analysis plot is set false
                            for i in range(9):                   # iteration
                                cv2.imshow('cube', frame)        # shows the frame 
                                cv2.waitKey(10)                  # refresh time is minimized (1ms), yet real time is much higher
                            time.sleep(vnc_delay)                # delay for cube face change, to compensate VNC viewer delay
                    
                    robot_to_cube_side(side, cam_led_bright)     # cube is rotated/flipped to the next face
                    t_ref = time.time()                          # timer is reset (used on each face detection to eventually use fix coordinates)

                    if side < 6:                                 # actions when a face has been completely detected, and there still are other to come
                        side +=1                                 # cube side index is incremented
                        break                                    # with this break the process re-starts from contour detection at the next cube face

                    if side == 6:                                # case last cube's face is acquired
                        disp.clean_display()                     # cleans the display
                        servo.cam_led_Off()                      # led at top_cover is set off         
                        cube_detect_time = time.time()           # time stored after detecteing all the cube facelets
                        if screen:                               # case screen variable is set true on __main__
                            try:                                 # tentative
                                cv2.destroyAllWindows()          # cube window and eventual other open windows are closed
                            except:                              # in case of exceptions
                                pass                             # do nothing
                        
                        # cube string status with colors detected 
                        cube_status, HSV_detected, cube_color_seq, HSV_analysis = cube_colors_interpr(URFDLB_facelets_BGR_mean)
                        cube_status_string = cube_string(cube_status)                 # cube string for the solver
                        solution, solution_Text = cube_solution(cube_status_string)   # Kociemba solver is called to have the solution string
                        color_detection_winner='BGR'                                  # variable used to log which method gave the solution
                        cube_solution_time=time.time()                                # time stored after getting the cube solution
                        print(f'\nCube status (via BGR color distance): {cube_status_string}')   # feedback is printed to the terminal
                        print(f'\nCamera warm-up, camera setting, cube status (BGR), and solution, in: {round(time.time()-start_time,1)} secs')
                        print()
                        
################### DEBUG #############             
#                         solution_Text = 'Error' # uncoment this row to force HSV color analysis method (this assignes 'Error' to solution_Text variable)
#######################################             

                        if 'Error' in solution_Text:               # if colors interpretation on BGR color distance fail an attempt is made on HSV
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

      
                        elif '0 moves' not in solution_Text:               # case of interest, the cube isn't already solved
                            print(f'\nCube solution: {solution_Text}')     # nice information to print at terminal, sometime useful to copy 
                        
                        if fcs == 0:   # (fcs = fix coordinates system) case the all facelets were detected without the fix coordinates method
                            save_coordinates(all_coordinates)              # saves the coordinates of the 9 facelets found during scanning

                        # function related to cube solving via the robot
                        robot_solve_cube(fixWindPos, screen, frame, faces, cube_status, cube_color_seq, HSV_analysis, 
                                            URFDLB_facelets_BGR_mean, font, fontScale, lineType, show_time, timestamp,
                                            solution, solution_Text, color_detection_winner, cube_status_string, BGR_mean,
                                            HSV_detected, start_time, camera_ready_time, cube_detect_time,
                                            cube_solution_time, os_version) 
                        
                        return              # closes the cube reading/solver function in case it reaches the end
                
                
                # case there are less than 9 contours detected, yet shown on screen as feedback
                if screen and not robot_stop:        # case screen variable is set true on __main__
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
    
    global camera, width, height, robot_stop, robot_idle, timeout
    global Rpi_ZeroW, cycles_num, picamera_test, quit_script

    import sys, time
    
    
    ################    general settings on how the robot is operated ###############################
    debug = False           # flag to enable/disable the debug related prints
    if args.debug != None:  # case 'debug' argument exists
        if args.debug:      # case the Cubotino_T.py has been launched with 'debug' argument
            debug = True    # flag to enable/disable the debug related prints is set True

    flip_to_close_one_step = True   # flag to enable/disable the flip_to_close in one step
    if args.twosteps != None: # case 'twosteps' argument exists
        if args.twosteps:   # case the Cubotino_T.py has been launched with 'fast' argument
            flip_to_close_one_step = False   # flag to enable/disable flip_to_close in one one step is set False
            
    screen = True           # flag to enable/disable commands requiring a screen connection, for graphical print out
    fixWindPos = True       # flag to fix the CV2 windows position, starting from coordinate 0,0 (top left)
    
    cv_wow = False          # flag to enable/disable the visualization of the image analysis used to find the facelets
    if args.cv_wow != None: # case 'cv_wow' argument exists
        if args.cv_wow:     # case the Cubotino_T.py has been launched with 'cv_wow' argument
            cv_wow = True   # flag to enable/disable the visualization of the image analysis used to find the facelets is set True
    if cv_wow:              # case the cv image analysis plot is set true
        screen = True       # screen related functions are activated
    
    fahrenheit = False      # flag to use fahrenheit degrees instead of celsius
    if args.F_deg != None:  # case 'F_deg' argument exists
        if args.F_deg:      # case the Cubotino_T.py has been launched with 'F_deg' argument
            fahrenheit = True  # flag to use fahrenheit degrees (instead of celsius) is set true
    
    timer = False           # flag to visualize a timer on screen after scrambling
    if args.timer != None:  # case the --timer argument has been prexists
        if args.timer:      # case the --timer argument has been provided
            timer = True    # lag to visualize a timer on screen after scrambling is set true
            
    picamera_test = False             # flag to enable/disable the PiCamera test
    if args.picamera_test != None:    # case 'picamera_test' argument exists
        if args.picamera_test:        # case the Cubotino_T.py has been launched with 'picamera_test' argument
            picamera_test = True      # flag to enable/disable the PiCamera test is set True

    btn = True                        # flag to enable/disable the start button at first cycle
    if args.no_btn != None:           # case 'no_btn' argument exists
        if args.no_btn:               # case the Cubotino_T.py has been launched with 'no_btn' argument
            btn = False               # flag to enable/disable the button at first cycle is set False
            
    
    silent = False                    # flag to enable/disable servos
    if args.silent != None:           # case 'silent' argument exists
        if args.silent:               # case the Cubotino_T.py has been launched with 'silent' argument
            silent = True             # flag to enable/disable the servos is set True
    
    clear_terminal()                  # cleares the terminal
    
    print('\nGeneral settings:')            # feedback is printed to the terminal
    if debug:                               # case the debug print-out are requested
        print(f'Debug prints activated\n')  # feedback is printed to the terminal
    else:                                   # case the debug print-out are not requested
        print(f'Debug prints not activated\n') # feedback is printed to the terminal
    
    param_imported, settings = import_parameters(debug) # imports the parameter from a json file
    if not param_imported:                  # case the function import_parameters returns False
        quit_func(quit_script=True)         # qutting function is called, with script clossure
    # ##############################################################################################



    ################    Pigpiod setting       ######################################################
    from Cubotino_T_pigpiod import pigpiod as pigpiod # start the pigpiod server
    # ##############################################################################################
    
    
    ################    Display setting       ######################################################
    from Cubotino_T_display import display as disp # sets the display object (the one on the robot)             
    disp.clean_display()                    # cleans the display
    # ##############################################################################################
        
    
    
    ################    Python version info    #####################################################
    try:                                    # tentative approach
        print(f'Python version: {sys.version}')   # print to terminal the python version
    except:                                 # case an exception is raised
        pass                                # no actions
    # ###############################################################################################
    

        
    ################    processor version info    ###################################################
    # when Rpi_ZeroW it uses slightly different openCV comands to prevent crashing (not Zero2W)
    import os                               # os is imported to check the machine
    Rpi_ZeroW = False                       # flag of a lighter program (OK on Rpi 3, 4 and Zero2)
    processor = ''                          # processor string variable is set empty
    try:                                    # tentative approach
        processor = os.uname().machine      # processor is inquired
        print(f'Processor architecture: {processor}')  # print to terminal the processor architecture
        if 'armv6' in processor:            # case the string armv6 is contained in processor string
            Rpi_ZeroW = True                # flag for program running on Rpi ZeroW
            print('Program adapted for armv6')  # feedback is printed to the terminal
    except:                                 # case an exception is raised
        pass                                # no actions
    # ###############################################################################################

     
    
    ################    OS version info    ##########################################################
    os_version = get_os_version()          # OS is checkjed
    if os_version==10:                     # case os_version equals 10
        print(f'Operative System found: Buster (10)') # print to terminal the OS version detected
        os_ver_txt = 'Buster'
    elif os_version==11:                   # case os_version equals 11
        print(f'Operative System found: Bullseye (11)')  # print to terminal the OS version detected
        os_ver_txt = 'Bullseye'
    else:                                  # other cases
        line = "#"*80
        print('\n\n')
        print(line)
        print('  The detected OS is not listed on those compatible with Cubotino  ')
        print(line)
        print('\n\n')
    
    script_v = version[:3]
    disp.set_backlight(1)                  # display backlight is turned on, in case it wasn't
    disp.show_on_display('OS: '+ os_ver_txt, 'Script V:'+ script_v, fs1=18, fs2=18) #feedbak is print to to the display
    time.sleep(3)                          # time delay to let possible readig the display
    # ###############################################################################################
    
    
    
    ################    screen presence, a pre-requisite for graphical   ############################
    screen_presence = check_screen_presence()             # checks if a screen is connected (also via VNC)
    
    if debug:                                             # case the debug print-out are requested
        print("Screen_presence: ",screen_presence)        # feedback is printed to the terminal
    
    if not screen_presence:                               # case there is not a screen connected 
        print(f'Screen related function are not activated')   # feedback is printed to the terminal 
        debug = False                                     # debug flag is set false
        disp.set_backlight(1)                             # display backlight is turned on, in case it wasn't
        disp.show_on_display('EXT. SCREEN', 'ABSENCE', fs1=16, fs2=20)  #feedbak is print to to the display
        time.sleep(2)                                     # time delay to let possible readig the screen
        screen = False                                    # screen flag is forced false
    
    if screen_presence:                                   # case there is a screen connected 
        disp.set_backlight(1)                             # display backlight is turned on, in case it wasn't
        disp.show_on_display('EXT. SCREEN', 'PRESENCE', fs1=16, fs2=19 )  #feedbak is print to to the display
        time.sleep(2)
        print('Screen related function are activated')    # feedback is printed to the terminal 
        if fixWindPos:                                    # case the graphical windows is forced to the top left monitor corner
            print('CV2 windows forced to top-left screen corner')    # feedback is printed to the terminal     
        
        if flip_to_close_one_step:                        # case the flip_to_close_one_step is set True
            print('From Flip-Up to close_cover in one continuos movement') # feedback is printed to the terminal
        else:                                             # case the flip_to_close_one_step is set False
            print('From Flip-Up to close_cover in two movements') # feedback is printed to the terminal
        
        if silent:                                        # case the silent variable is set True
            print('Servos deactivated after init')        # feedback is printed to the terminal
        else:                                             # case the silent variable is set False
            print('Servos are activated')                 # feedback is printed to the terminal
            
        if cv_wow:                                        # case the cv image analysis plot is set true
            print('CV image analysis is plot on screen')  # feedback is printed to the terminal 
        
        if timer:                                         # case the timer visualization is set true
            print('Timer is visualized after scrambling function')   # feedback is printed to the terminal 
        
        if frameless_cube == 'false':                               # case the frameless string variale equals to false
            print('\nCube status detection set for cube with black frame around the facelets')  # feedback is printed to the terminal 
        elif frameless_cube == 'true':                              # case the frameless string variale equals to 'true'
            print('\nCube status detection set for frameless cube') # feedback is printed to the terminal
        elif frameless_cube == 'auto':                              # case the frameless string variale equals to 'auto'
            print('\nCube status detection set for both cubes with and without black frame')   # feedback is printed to the terminal 
            print('This setting takes slightly longer time for the cube status detection\n')   # feedback is printed to the terminal
        
        if picamera_test:                                 # case picamera_test is set true (servos disabling)
            print(f'\nPiCamera test enabled')             # feedback is printed to the terminal
            test_picamera()                               # test_picamera func is called

        if not btn:                                       # case the btn is set False
            print(f'\nButton is ignored, solving cycles start automatically')   # feedback is printed to the terminal
    # ###############################################################################################
    
    
    
    ###################################    import libraries    ######################################
    print('\nImport libraries:')            # feedback is printed to the terminal    
    import_libraries()                      # imports libraries
    # ###############################################################################################
    
    
    
    #################################    startup  variables     #####################################
    print('\nOther settings and environment status:')  # feedback is printed to the terminal
    cycles_num = 0                          # zero is assigned to the (automated) cycles_num variable
    start_up(first_cycle = True)            # sets the initial variables, in this case it is the first cycle
    solv_cycle = 0                          # variable to count the solving cycles per session is set to zero
    scramb_cycle = 0                        # variable to count the scrambling cycles per session is set to zero
    quit_script = False
    print('\n#############################################################################\n')
    # ###############################################################################################
    
    

    ###########################  parsing arguments for robot remote usage  ##########################
    if args.cycles != None:                 # case the Cubotino_T.py has been launched with 'cycles' argument
        cycles_num = abs(int(args.cycles))  # the positive integer arg passed is assigned to the cycle_num variable
        if cycles_num > 0:                  # case the automated cycles request is more than zero
            print(f'\nAsked the robot to automatically scramble and solve the cube {cycles_num} times')
            automated = True                # automated variable is set true
        else:                               # case the automated cycles request equals zero
            automated = False               # automated variable is set false
    else:                                   # case the Cubotino_T.py has not been launched without 'cycles' argument
        automated = False                   # automated variable is set false
    
    if cycles_num > 0 and args.pause != None:  # case the Cubotino_T.py has been launched also with 'pause' argument
        
        # the positive integer arg passed, rounded to the closest upper multiple of 4, is assigned to the cycle_pause
        cycle_pause = 4*int(math.ceil(abs(int(args.pause))/4))  
        print(f'Asked the robot to pause {cycle_pause} seconds in between the automated cycles\n') 
    else:                                   # case the Cubotino_T.py has not been launched without 'pause' argument
        cycle_pause = 0                     # zero is assigned to the cycles_num variable
    # ###############################################################################################
        
    

    while True:                             # (outer) infinite loop, until the Rpi is shut-off
        robot_stop = False                  # flag used to stop or allow robot movements
        robot_idle = True                   # robot is idling
        timeout = False                     # flag used to limit the cube facelet detection time
        
        while not automated:                # while automated is false: (inner) infinite loop, until 'solve' process is chosen
            if btn:                         # case the variable btn is set True (touch button enables)
                print('\n\n\n\nWaiting for user to start a cycle')  # feedback is printed to the terminal
                cycle = press_to_start()    # request user to press the button to start a scrambling or solving cycle
            elif not btn:                   # case the variable btn is set False (testing without the touch button)
                print('\n\n\n\nCycle is started without using the touch sensor')  # feedback is printed to the terminal
                cycle = 'solve'             # string 'solve' is returned to start a solving cycle
            
            if cycle == 'scramble':         # case the chosen cycle is cube scrambling
                # scramble can be done more times within this inner while loop
                scramb_cycle += 1           # counter, for the number of scrambling cycles perfomed within a session, is incremented
                start_scrambling(scramb_cycle)  # start_scrambling function is called
            
            elif cycle == 'solve':          # case the chosen cycle is cube scrambling
                solv_cycle += 1             # counter, for the number of solving cycles perfomed within a session, is incremented
                start_solving(solv_cycle)   # start_solving function is called
                start_up(first_cycle = False)  # sets the initial variables, to use the camera in manual mode
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
                    camera, width, height = set_camera()  # camera has to be re-initialized, to removes previous settings
                
                if i+1 == cycles_num:           # case all the cycles have been done
                    # closing the automated cycles section
                    print('\n\n\n\n\n\n')       # prints some empty lines, on IDE terminal
                    clear_terminal()            # cleares the terminal
                    print(f'\nScrambled and solved the cube {cycles_num} times\n')
                    solv_cycle = cycles_num     # cycle_num is assigned to variable counting the solving cycles manually requested
                    scramb_cycle = cycles_num   # cycle_num is assigned to variable counting the scrambling cycles manually requested
                    
                    if args.shutoff:                  # case the --shutoff argument has been provided
                        quit_func(quit_script = True) # the script is terminated and, depending on Cubotino_T_bash.sh, it might shut the RPI off
                    else:                             # case the --shutoff argument has not been provided
                        automated = False             # automated variable is set false, robot waits for solve button commands
                
                start_up(first_cycle = False)  # sets the initial variables, to use the camera in manual mode
