#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
# Andrea Favero 30 June 2022
#
# This script relates to CUBOTino autonomous, a very small and simple Rubik's cube solver robot 3D printed
# CUBOTino autonomous is the 'Top version', of the CUBOTino versions
# This specific script controls two servos based on the movements string from Cubotino.py
# 
# Possible moves with this robot
# 1) Spins the complete cube ("S") laying on the bottom face: 1 means CW 90deg turns, while 3 means 90CCW turn
# 2) Flips the complete cube ("F") by "moving" the Front face to Bottom face: Only positive values are possible
# 3) Rotates the bottom layer ("R") while costraining the 2nd and 3rd layer.
# 4) The order of S, F has to be strictly followed
# 5) Example 'F1R1S3' means: 1x cube Flip, 1x (90deg) CW rotation of the 1st (Down) layer, 1x (90deg) CCW cube Spin
#
# For Rotations, the bottom servo makes a little extra rotation than target, before coming back to target; This
# is needed to recover the gaps between cube_hoder - cube - top_cover, and still getting a decent cube layers alignment.
#
#
# Developped on:
#  - Raspberry pi Zero 2 
#  - Raspberry Pi OS (Legacy) A port of Debian Buster with security updates and desktop environment
#    (Linux raspberry 5.10.103-v7+ #1529 SMP Tue 8 12:21:37 GMT 2022 armv71 GNU/Linux)
#
#############################################################################################################
"""

##################    imports for the display part   ################################
from PIL import Image, ImageDraw #, ImageFont
from Cubotino_T import set_display                # function to generate the display object
from Cubotino_T import show_cubotino              # function to show Cubotino logo
from Cubotino_T import display_progress_bar       # function to show the progress on display while the servos are operated

s_disp = set_display()                            # display object creation
s_disp.set_backlight(0)                           # display backlight is set off
s_disp_w = s_disp.width                           # display width
s_disp_h = s_disp.height                          # display height
s_disp_img = Image.new('RGB', (s_disp_w, s_disp_h), color=(0, 0, 0)) # display image generation, fullt black
s_disp_draw = ImageDraw.Draw(s_disp_img)          # display graphic generation

import os.path, pathlib
folder = pathlib.Path().resolve()                 # active folder (should be home/pi/cube)
fname = os.path.join(folder,'Cubotino_T_Logo_265x212_BW.jpg')   # folder and file name for the jpg file of Cubotino_logo
jpg_logo = False                                  # jpg_logo variable is set false
if os.path.exists(fname):                         # case the jpg file of Cubotino_logo exists
    jpg_logo = True                               # jpg_logo variable is set true
show_cubotino(delay=None, jpg_logo=jpg_logo)      # show cubotino logo on display, when this script is imported
s_disp.set_backlight(1)                             # activates the display backlight
# ##################################################################################





##################    imports and servo_settings for Servos and LED   ####################
import time                                       # import time library
import RPi.GPIO as GPIO                           # import RPi GPIO library
GPIO.setmode(GPIO.BCM)                            # setting GPIO pins as "Broadcom SOC channel" number, these are the numbers after "GPIO"
GPIO.setwarnings(False)                           # setting GPIO to don't return allarms
from gpiozero import Servo, PWMLED                # import modules for the PWM part
from gpiozero.pins.pigpio import PiGPIOFactory    # pigpio library is used, to use hardare timers on PWM (to avoid servo jitter)
factory = PiGPIOFactory()                         # pin factory setting to use hardware timers, to avoid servo jitter


# servo_settings for the led on top cover, to ensure sufficient light while the PiCamera is reading
top_cover_led_pin = 18                            # GPIO pin used to control the LED on/off/intensity
# NOTE: High frequency on the led PWM is needed to prevent the camera to see the led flickering
top_cover_led = PWMLED(top_cover_led_pin, active_high=True, initial_value=0, frequency=5000000, pin_factory=factory)

# overall servo_settings for the GPIO and the PWM
t_servo_pin = 12                  # GPIO pin used for the top servo
b_servo_pin = 13                  # GPIO pin used for the bottom servo
# ##################################################################################



##################    general servo_settings    #########################################
robot_init_status=False         # boolean to track the servos inititialization status
stop_servos=True                # boolean to stop the servos during solving proces: It is set true at the start, servos cannot operate
b_servo_operable=False          # variable to block/allow bottom servo operation
fun_status=False                # boolean to track the robot fun status, it is True after solving the cube :-)
s_debug=False                   # boolean to print out info when debugging
# ##################################################################################




def init_servo(print_out=s_debug):
    """ Function to initialize the robot (servos position) and some global variables, do be called once, at the start.
        Parameters are imported from a json file, to make easier to list/document/change the variables
        that are expected to vary on each robot.
        These servo_settings are under a function, instead of root, to don't be executed when this script is used from CLI
        with arguments; In this other case the aim is to help setting the servo to their mid position."""
    
    global t_servo, t_servo_close, t_servo_open, t_servo_read, t_servo_flip, t_servo_rel, t_top_cover    # top servo related angles and status
    global t_flip_to_close_time, t_close_to_flip_time, t_flip_open_time, t_open_close_time               # top servo related timers
    global b_servo, b_servo_CCW, b_servo_CW, b_home, b_extra_sides, b_extra_home                         # bottom servo related angles
    global b_servo_CCW_rel, b_servo_CW_rel, b_home_from_CCW, b_home_from_CW                              # bottom servo related angles
    global b_servo_home, b_servo_stopped, b_servo_CW_pos, b_servo_CCW_pos                                # bottom servo related status 
    global b_spin_time, b_rotate_time, b_rel_time                                                        # bottom servo related timers 
    global robot_init_status, fun_status, led_init_status                                                # bottom servo related angles 
    
    if not robot_init_status:                                            # case the inititialization status of the servos false
        
        # convenient choice for Andrea Favero, to upload the settings fitting my robot, via mac address check                
        from getmac import get_mac_address                               # library to get the device MAC ddress
        import os.path, pathlib, json                                    # libraries needed for the json, and parameter import
        
        folder = pathlib.Path().resolve()                                # active folder (should be home/pi/cube)  
        eth_mac = get_mac_address()                                      # mac address is retrieved
        if eth_mac == 'e4:5f:01:8d:59:97':                               # case the script is running on AF (Andrea Favero) robot
            fname = os.path.join(folder,'Cubotino_T_servo_settings_AF.txt') # AF robot settings (do not use these at the start)
        else:                                                            # case the script is not running on AF (Andrea Favero) robot
            fname = os.path.join(folder,'Cubotino_T_servo_settings.txt') # folder and file name for the settings, to be tuned
 
        if os.path.exists(fname):                                        # case the servo_settings file exists
            with open(fname, "r") as f:                                  # servo_settings file is opened in reading mode
                servo_settings = json.load(f)                            # json file is parsed to a local dict variable
                # NOTE: in case of git pull, the settings file will be overwritten, the backup file not
            
            backup_fname = os.path.join(folder,'Cubotino_T_servo_settings_backup.txt')     # folder and file name for the settings backup
            with open(backup_fname, 'w') as f:                           # servo_settings_backup file is opened in writing mode
                f.write(json.dumps(servo_settings, indent=0))            # content of the setting file is saved in another file, as backup
                # NOTE: in case of git pull, the settings file will be overwritten, the backup file not
            
            try:
                t_min_pulse_width = float(servo_settings['t_min_pulse_width'])        # defines the min Pulse With the top servo reacts to
                t_max_pulse_width = float(servo_settings['t_max_pulse_width'])        # defines the max Pulse With the top servo reacts to
                t_servo_close = float(servo_settings['t_servo_close'])                # Top_cover close position, in gpiozero format
                t_servo_open = float(servo_settings['t_servo_open'])                  # Top_cover open position, in gpiozero format
                t_servo_read = float(servo_settings['t_servo_read'])                  # Top_cover camera read position, in gpiozero format
                t_servo_flip = float(servo_settings['t_servo_flip'])                  # Top_cover flip position, in gpiozero format
                t_servo_rel_delta = float(servo_settings['t_servo_rel_delta'])        # Top_cover release angle movement from the close position to release tension
                t_flip_to_close_time = float(servo_settings['t_flip_to_close_time'])  # time for Top_cover from flip to close position
                t_close_to_flip_time = float(servo_settings['t_close_to_flip_time'])  # time for Top_cover from close to flip position 
                t_flip_open_time = float(servo_settings['t_flip_open_time'])          # time for Top_cover from open to flip position, and viceversa  
                t_open_close_time = float(servo_settings['t_open_close_time'])        # time for Top_cover from open to close position, and viceversa
                
                b_min_pulse_width = float(servo_settings['b_min_pulse_width'])        # defines the min Pulse With the bottom servo reacts to
                b_max_pulse_width = float(servo_settings['b_max_pulse_width'])        # defines the max Pulse With the bottom servo reacts to
                b_servo_CCW = float(servo_settings['b_servo_CCW'])                    # Cube_holder max CCW angle position
                b_servo_CW = float(servo_settings['b_servo_CW'])                      # Cube_holder max CW angle position
                b_home = float(servo_settings['b_home'])                              # Cube_holder home angle position
                b_extra_sides = float(servo_settings['b_extra_sides'])                # Cube_holder release angle from CCW and CW angle positions, to release tension
                b_extra_home = float(servo_settings['b_extra_home'])                  # Cube_holder release angle from home angle positions, to release tension
                b_spin_time = float(servo_settings['b_spin_time'])                    # time for Cube_holder to spin 90 deg (cune not contrained)
                b_rotate_time = float(servo_settings['b_rotate_time'])                # time for Cube_holder to rotate 90 deg (cube constrained)
                b_rel_time = float(servo_settings['b_rel_time'])                      # time for Cube_holder to release tension at home, CCW and CW positions
            
                
            except:   # exception will be raised if json keys differs, or parameters cannot be converted to float
                print('error on converting to float the imported parameters')   # feedback is printed to the terminal                                  
                return robot_init_status                                        # return robot_init_status variable, that is False
        
        else:                                                                   # case the servo_settings file does not exists, or name differs
            print('could not find Cubotino_T_servo_settings.txt')               # feedback is printed to the terminal                                  
            return robot_init_status                                            # return robot_init_status variable, that is False
        
        # t_servo servo object creation
        t_servo = Servo(t_servo_pin,                                 # GPIO pin associated to the top servo
                        initial_value = t_servo_open,                # Top_cover positioned in open position
                        min_pulse_width = t_min_pulse_width/1000,    # min Pulse Width the top servo reacts to
                        max_pulse_width = t_max_pulse_width/1000,    # max Pulse Width the top servo reacts to
                        pin_factory=factory)                         # way to use hardware based timers for the PWM 
        time.sleep(0.7)                                              # wait time to ensure the servo to get its init position
        t_top_cover = 'open'                                         # variable to track the top cover/lifter position
        
        # b_servo servo object creation
        b_servo = Servo(b_servo_pin,                                 # GPIO pin associated to the bottom servo
                        initial_value = b_home,                      # Cube_holder positioned to home
                        min_pulse_width = b_min_pulse_width/1000,    # min Pulse Width the bottom servo reacts to
                        max_pulse_width = b_max_pulse_width/1000,    # max Pulse Width the bottom servo reacts to
                        pin_factory=factory)                         # way to used hardware based timers for the PWM                   
        time.sleep(0.7)                                              # wait time to ensure the servo to get its init position
        b_servo.value = b_home                                       # bottom servo is set to home postion at the start
        b_servo_home=True                                   # boolean of bottom servo at home
        b_servo_stopped = True                              # boolean of bottom servo at location the lifter can be operated
        b_servo_CW_pos=False                                # boolean of bottom servo at full CW position
        b_servo_CCW_pos=False                               # boolean of bottom servo at full CCW position


        # bottom servo derived positions
        b_servo_CCW_rel = b_servo_CCW + b_extra_sides       # bottom servo position to rel tensions when fully CW
        b_servo_CW_rel = b_servo_CW - b_extra_sides         # bottom servo position to rel tensions when fully CCW
        b_home_from_CW = b_home - b_extra_home              # bottom servo extra home position, when moving back from full CW
        b_home_from_CCW = b_home + b_extra_home             # bottom servo extra home position, when moving back from full CCW
        
        # top servo derived position
        t_servo_rel = t_servo_close - t_servo_rel_delta     # top servo position to release tension

        robot_init_status = True          # boolean to track the inititialization status of the servos is set true
        if print_out:
            print("servo init done")
    
    
    servo_start_pos()                 # servos are positioned to the start position    
    fun_status=False                  # boolean to track the robot fun status, it is True after solving the cube :-)
    servo_off()                       # PWM signal at GPIO is stopped
    cam_led_test()                    # makes a very short led test
    cam_led_Off()                     # forces the led off
    
    return robot_init_status







def quit_func():
    """ Sets the used GPIO as output, and force them to low. This wants to prevent servos to move after closing the script."""
    
    GPIO.setup(top_cover_led_pin, GPIO.OUT, initial=GPIO.LOW)   # GPIO top_cover_led_pin (Top_cover_Led) is set as output, and force low
    GPIO.setup(t_servo_pin, GPIO.OUT, initial=GPIO.LOW)         # GPIO t_servo_pin (Top_cover servo) is set as output, and force low
    GPIO.setup(b_servo_pin, GPIO.OUT, initial=GPIO.LOW)         # GPIO b_servo_pin (Cube_holder servo) is set as output, and force low







def cam_led_On(value=0.05):
    """ Sets the top_cover led ON, with brightness in arg. Value ranges from 0 to 0.3"""
    
    if value >= 0.3:                              # case arg is bigger than, or equal to 0.3
        top_cover_led.value = 0.3                 # top cover led PWM is set to 30%
    else:                                         # case arg is smaller than 0.3
        top_cover_led.value = value               # top cover led PWM is set to arg value






def cam_led_Off():
    """ Sets the top_cover led OFF."""
    
    top_cover_led.off()      # top cover led PWM is set to 0







def cam_led_test():
    """ Fade the led On and Off at the init, to show the led worwing."""
    
    for i in range(0,32,4):            # iterates from 0 to 28 in steps of 4 
        top_cover_led.value = i/100    # top cover led PWM is set iterator %
        time.sleep(0.02)               # very short time sleep
    
    for i in range(28,-4,-4):          # iterates from 28 to 0 in steps of -4 
        top_cover_led.value = i/100    # top cover led PWM is set iterator %
        time.sleep(0.02)               # very short time sleep






def servo_start_pos():
    """Servos are positioned to the intended start position, meaning top_cover on read position and cube_holder on its middle position"""
    
    global t_servo_open, t_top_cover
    global b_servo_operable, b_servo_stopped,  b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    global stop_servos
    
    stop_servos=False                 # boolean to stop the servos during solving process is set False: servos can be operated
    b_servo_operable=False            # variable to block/allow bottom servo operation
    
    
    t_servo.value = t_servo_open      # top servo is positioned in open position
    time.sleep(t_close_to_flip_time)  # time to allow the top_cover/lifter to reach the open postion   
    t_top_cover = 'open'              # variable to track the top cover/lifter position
    b_servo_operable=True             # variable to block/allow bottom servo operation
    
    
    b_servo.value = b_home            # bottom servo moves to the home position, releasing then the tensions
    time.sleep(b_rotate_time)         # time for the servo to release the tensions
    
    b_servo_stopped = True            # boolean of bottom servo at location the lifter can be operated
    b_servo_home=True                 # boolean of bottom servo at home
    b_servo_CW_pos=False              # boolean of bottom servo at full CW position
    b_servo_CCW_pos=False             # boolean of bottom servo at full CCW position
    
    b_servo_operable=False            # variable to block/allow bottom servo operation
    t_servo.value = t_servo_read      # top servo is positioned in read position at the start
    time.sleep(t_flip_open_time)      # time to allow the top_cover/lifter to reach the read postion
    t_top_cover = 'read'              # variable to track the top cover/lifter position
    
    cam_led_Off()                     # forces the top_cover_led to off






def servo_off():
    """ Function to stop sending the PWM the servos."""
    
#     t_servo.detach()             # PWM is stopped at the GPIO pin for top servo
#     b_servo.detach()             # PWM is stopped at the GPIO pin for bottom servo
    return







def stopping_servos(print_out=s_debug):
    """ Function to stop the servos."""
    global stop_servos
    
    if print_out:
        print("\ncalled the servos stopping function\n")    
    stop_servos=True                      # boolean to stop the servos during solving process, is set true: Servos are stopped







def stop_release(print_out=s_debug):
    """ Function to relelease the stop from servos."""
    
    global stop_servos
    
    if print_out:
        print("\ncalled the stop release function\n")
    stop_servos=False            # boolean to stop the servos during solving process, is set false: Servo can be operated







def read():
    """ Function to position the top_cover to the read."""
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                          # case there is not a stop request for servos
        if b_servo_stopped==True:                # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False               # variable to block/allow bottom servo operation
            if t_top_cover == 'close':           # cover/lifter position variable set to close
                t_servo.value = t_servo_read     # servo is positioned to flip the cube
                time.sleep(t_close_to_flip_time) # time for the servo to reach the flipping position from close position
            elif t_top_cover == 'open':          # cover/lifter position variable set to open
                t_servo.value = t_servo_read     # servo is positioned to flip the cube
                time.sleep(t_flip_open_time)     # time for the servo to reach the flipping position                
            t_top_cover='read'                   # cover/lifter position variable set to flip
            servo_off()                          # PWM signal at GPIO is stopped 







def flip_up():
    """ Function to raise the flipper to the upper position, to flip the cube around its horizontal axis."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                          # case there is not a stop request for servos
        if b_servo_stopped==True:                # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False               # variable to block/allow bottom servo operation
            if t_top_cover == 'close':           # cover/lifter position variable set to close
                t_servo.value = t_servo_flip     # servo is positioned to flip the cube
                time.sleep(t_close_to_flip_time) # time for the servo to reach the flipping position from close position
            elif t_top_cover == 'open' or t_top_cover == 'read':   # cover/lifter position variable set to open or read positions
                t_servo.value = t_servo_flip     # servo is positioned to flip the cube
                time.sleep(t_flip_open_time)     # time for the servo to reach the flipping position                
            t_top_cover='flip'                   # cover/lifter position variable set to flip
            servo_off()                          # PWM signal at GPIO is stopped 






def flip_to_open():
    """ Function to raise the top cover to the open position. The cube is not contrained by the top cover or the flipper."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                          # case there is not a stop request for servos
        if b_servo_stopped==True:                # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False               # variable to block/allow bottom servo operation
#             t_servo.value = t_servo_close        # servo is positioned to constrain the mid and top cube layers
#             time.sleep(t_flip_to_close_time)     # time for the servo to reach the close position from the flip position
            t_servo.value = t_servo_open         # top servo is positioned in open top cover position, from close position
            time.sleep(t_flip_open_time)         # time for the top servo to reach the open top cover position
            t_top_cover='open'                   # variable to track the top cover/lifter position
            b_servo_operable=True                # variable to block/allow bottom servo operation
            servo_off()                          # PWM signal at GPIO is stopped 






def flip_to_close():
    """ Function to lower the flipper to the close position, position that contrains the cube with the top cover."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                            # case there is not a stop request for servos
        if b_servo_stopped==True:                  # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False                 # variable to block/allow bottom servo operation
            t_servo.value = t_servo_close          # servo is positioned to constrain the mid and top cube layers
            if t_top_cover == 'flip' or t_top_cover == 'read':  # cover/lifter position variable set to flip
                time.sleep(t_flip_to_close_time)   # time for the servo to reach the close position
            elif t_top_cover == 'open':            # cover/lifter position variable set to open or read positions
                time.sleep(t_open_close_time)      # time for the servo to reach the flipping position
            t_servo.value = t_servo_rel            # servo is positioned to release the tention from top of the cube (in case of contact)
            t_top_cover='close'                    # cover/lifter position variable set to close
            b_servo_operable=True                  # variable to block/allow bottom servo operation
            servo_off()                            # PWM signal at GPIO is stopped 






def flip():
    """ Flips the cube during the cube detection phase, and places the top_cover (piCamera) in read position."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    
    flip_up()
    
    if not stop_servos:                        # case there is not a stop request for servos
        if b_servo_stopped==True:              # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False             # variable to block/allow bottom servo operation
            t_servo.value = t_servo_read       # top servo is positioned in top cover read position
            time.sleep(t_flip_open_time+0.1)   # time for the top servo to reach the top cover read position
            t_top_cover='read'                 # variable to track the top cover/lifter position
            b_servo_operable=False             # variable to block/allow bottom servo operation
            servo_off()                        # PWM signal at GPIO is stopped 






def open_cover():
    """ Function to open the top cover from the close position, to release the contrain from the cube."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped
    
    if not stop_servos:                            # case there is not a stop request for servos
        if b_servo_stopped==True:                  # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False                 # variable to block/allow bottom servo operation
            t_servo.value = t_servo_open           # servo is positioned to open
            time.sleep(t_open_close_time)          # time for the servo to reach the open position
            t_top_cover='open'                     # variable to track the top cover/lifter position
            b_servo_operable=True                  # variable to block/allow bottom servo operation
            servo_off()                            # PWM signal at GPIO is stopped







def close_cover():
    """ Function to close the top cover, to contrain the cube."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    if not stop_servos:                            # case there is not a stop request for servos
        if b_servo_stopped==True:                  # boolean of bottom servo at location the lifter can be operated
            b_servo_operable=False                 # variable to block/allow bottom servo operation
            t_servo.value = t_servo_close          # servo is positioned to open
            time.sleep(t_open_close_time)          # time for the servo to reach the open position
            t_servo.value = t_servo_rel            # servo is positioned to release the tention from top of the cube (in case of contact)
            t_top_cover='close'                    # cover/lifter position variable set to close
            b_servo_operable=True                  # variable to block/allow bottom servo operation
            servo_off()                            # PWM signal at GPIO is stopped 






def spin_out(direction):
    """ Function that spins the cube holder toward CW or CCW.
        During the spin the cube is not contrained by the top cover.
        The cube holder stops to the intended position, without making extra rotation."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
           
    if not b_servo_operable and t_top_cover=='read':
        flip_to_open()
        
    if not stop_servos:                              # case there is not a stop request for servos
        if b_servo_operable==True:                   # variable to block/allow bottom servo operation
            if b_servo_home==True:                   # boolean of bottom servo at home
                b_servo_stopped=False                # boolean of bottom servo at location the lifter can be operated
                b_servo_CW_pos=False                 # boolean of bottom servo at full CW position
                b_servo_CCW_pos=False                # boolean of bottom servo at full CCW position
                
                if direction=='CCW':                 # case the set direction is CCW
                    b_servo.value = b_servo_CCW      # bottom servo moves to the most CCW position
                    time.sleep(b_spin_time)          # time for the bottom servo to reach the most CCW position
                    b_servo_CCW_pos=True             # boolean of bottom servo at full CCW position
                
                elif direction=='CW':                # case the set direction is CW
                    b_servo.value = b_servo_CW       # bottom servo moves to the most CCW position
                    time.sleep(b_spin_time)          # time for the bottom servo to reach the most CCW position
                    b_servo_CW_pos=True              # boolean of bottom servo at full CW position
                
                b_servo_stopped=True                 # boolean of bottom servo at location the lifter can be operated
                b_servo_home=False                   # boolean of bottom servo at home
                b_servo_stopped=True                 # boolean of bottom servo at location the lifter can be operated
                servo_off()                          # PWM signal at GPIO is stopped







def spin_home():
    """ Function that spins the cube holder to home position.
        During the spin the cube is not contrained by the top cover.
        The cube holder stops to home position, without making extra rotation."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    if not b_servo_operable and t_top_cover=='read':
        flip_to_open()
    
    if not stop_servos:                         # case there is not a stop request for servos
        if b_servo_operable==False:             # variable to block/allow bottom servo operation
            open_cover()                        # top servo is moved to open position
        elif b_servo_operable==True:            # variable to block/allow bottom servo operation
            if b_servo_home==False:             # boolean of bottom servo at home
                b_servo_stopped = False         # boolean of bottom servo at location the lifter can be operated
                b_servo.value = b_home          # bottom servo moves to the home position, releasing then the tensions
                time.sleep(b_spin_time)         # time for the bottom servo to reach the extra home position
                b_servo_stopped=True            # boolean of bottom servo at location the lifter can be operated
                b_servo_home=True               # boolean of bottom servo at home
                b_servo_CW_pos=False            # boolean of bottom servo at full CW position
                b_servo_CCW_pos=False           # boolean of bottom servo at full CCW position
                servo_off()                     # PWM signal at GPIO is stopped







def spin(direction):
    """ Spins the cube during the cube detection phase"""
    
    spin_out(direction)  # calls the spin_out function
    spin_home()          # calls the spin home function






def rotate_out(direction):
    """ Function that rotates the cube holder toward CW or CCW position; During the rotation the cube is contrained by the top cover.
        The cube holder makes first an extra rotation, and later it comes back to the intended position; This approach
        is used for a better facelets alignment to the faces, and to relese the friction (cube holder - cube - top cover)."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    
    if not stop_servos:                                   # case there is not a stop request for servos
        if t_top_cover!='close':                          # case the top cover is not in close position
            flip_to_close()                               # top cover is lowered in close position
        
        if b_servo_operable==True:                        # variable to block/allow bottom servo operation
            if b_servo_home==True:                        # boolean of bottom servo at home=
                b_servo_stopped=False                     # boolean of bottom servo at location the lifter can be operated
                
                if direction=='CCW':                      # case the set direction is CCW
                    b_servo.value = b_servo_CCW           # bottom servo moves to the most CCW position
                    time.sleep(b_rotate_time)             # time for the bottom servo to reach the most CCW position
                    b_servo.value = b_servo_CCW_rel       # bottom servo moves slightly to release the tensions
                    time.sleep(b_rel_time)                # time for the servo to release the tensions
                    b_servo_CCW_pos=True                  # boolean of bottom servo at full CCW position
                    
                elif direction=='CW':                     # case the set direction is CW
                    b_servo.value = b_servo_CW            # bottom servo moves to the most CCW position
                    time.sleep(b_rotate_time)             # time for the bottom servo to reach the most CCW position
                    b_servo.value = b_servo_CW_rel        # bottom servo moves slightly to release the tensions
                    time.sleep(b_rel_time)                # time for the servo to release the tensions
                    b_servo_CW_pos=True                   # boolean of bottom servo at full CW position
                    
                b_servo_stopped=True                      # boolean of bottom servo at location the lifter can be operated
                b_servo_home=False                        # boolean of bottom servo at home
                servo_off()                               # PWM signal at GPIO is stopped 
                
                if t_top_cover=='close':                  # case the top cover is in close position
                    open_cover()                          # top cover is raised in open position








def rotate_home(direction):
    """ Function that rotates the cube holder to home position; During the rotation the cube is contrained by the top cover.
        The cube holder makes first an extra rotation, and later it comes back to the home position; This approach
        is used for a better facelets alignment to the faces, and to relese the friction (cube holder - cube - top cover)."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    if not stop_servos:                                # case there is not a stop request for servos
        if b_servo_operable==True:                     # variable to block/allow bottom servo operation
            if b_servo_home==False:                    # boolean of bottom servo at home
                
                if t_top_cover!='close':               # case the top cover is not in close position
                    flip_to_close()                    # top cover is lowered in close position
                
                if direction=='CCW':                   # case the set direction is CCW
                    if b_servo_CW_pos==True:           # boolean of bottom servo at full CW position
                        b_servo.value = b_home_from_CW   # bottom servo moves to the extra home position, from CCW
                
                elif direction=='CW':                  # case the set direction is CW
                    if b_servo_CCW_pos==True:          # boolean of bottom servo at full CW position
                        b_servo.value = b_home_from_CCW  # bottom servo moves to the extra home position, from CW
                
                time.sleep(b_rotate_time)              # time for the bottom servo to reach the extra home position
                b_servo.value = b_home                 # bottom servo moves to the home position, releasing then the tensions
                time.sleep(b_rel_time)                 # time for the servo to release the tensions
                b_servo_stopped=True                   # boolean of bottom servo at location the lifter can be operated
                b_servo_home=True                      # boolean of bottom servo at home
                b_servo_CW_pos=False                   # boolean of bottom servo at full CW position
                b_servo_CCW_pos=False                  # boolean of bottom servo at full CCW position
                
                open_cover()                           # top cover is raised in open position







def check_moves(moves, print_out=s_debug):
    """ Function that counts the total servo moves, based on the received moves string.
        This function also verifies if the move string is compatible with servo contrained within 180 deg range (from -90 to 90 deg):
        Not possible to rotate twice +90deg (or -90deg) from the center, nor 3 times +90deg (or -90deg) from one of the two extremes."""
    
    servo_angle=0                                         # initial angle is set to zero, as this is the starting condition at string receival
    servo_angle_ok=True                                   # boolean to track the check result
    tot_moves=0                                           # counter for the total amount of servo moves (1x complete flip, 1x each 90 deg cube spin or 1st layer rotation)
    
    for i in range(len(moves)):                           # iteration over all the string characters
        if moves[i]=='1':                                 # case direction is CW 
            if moves[i-1] == 'R' or moves[i-1] == 'S':    # case the direction refers to cube spin or layer rotation
                servo_angle+=90                           # positive 90deg angle are added to the angle counter
                tot_moves+=1                              # counter is increased

        elif moves[i]=='3':                               # case direction is CW 
            if moves[i-1] == 'R' or moves[i-1] == 'S':    # case the direction refers to cube spin or layer rotation
                servo_angle-=90                           # negative 90deg angle are subtracted from the angle counter
                tot_moves+=1                              # counter is increased

        elif moves[i]=='F':                               # case there is a flip on the move string
            tot_moves+=int(moves[i+1])                    # counter is increased
        
        if servo_angle<-90 or servo_angle>180:            # case the angle counter is out of range
            if print_out:
                print(f'servo_angle out of range at string pos:{i}')  # info are printed
            servo_angle_ok=False                          # bolean of results is updated
            break                                         # for loop is interrupted
    
    if servo_angle_ok==True:                              # case the coolean is still positive
        if print_out:
            print('servo_angle within range')             # positive result is printed
    
    remaining_moves={}                                            # empty dict to store the left moves 
    left_moves=tot_moves                                          # initial remaining moves are all the moves
    for i in range(len(moves)):                                   # iteration over all the string characters
        if moves[i]=='R' or moves[i]=='S':                        # case the move is cube spin or layer rotation               
            left_moves-=1                                         # counter is decreased by one
            remaining_moves[i]=int(100*(1-left_moves/tot_moves))  # solving percentage associated to the move index key
        elif moves[i]=='F':                                       # case there is a flip on the move string
            left_moves-=int(moves[i+1])                           # counter is decreased by the amount of flips
            remaining_moves[i]=int(100*(1-left_moves/tot_moves))  # solving percentage associated to the move index key   
        
    return servo_angle_ok, tot_moves, remaining_moves






def fun(print_out=s_debug):
    """ Cube holder spins, to make some vittory noise once the cube is solved."""

    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home, b_servo_CW_pos, b_servo_CCW_pos
    
    if print_out:
        print("fun function")
    
    if t_top_cover != 'open':                        # variable to track the top cover/lifter position
        if not stop_servos:                          # case there is not a stop request for servos
            if b_servo_stopped==True:                # boolean of bottom servo at location the lifter can be operated
                b_servo_operable=False               # variable to block/allow bottom servo operation
                t_servo.value = t_servo_open         # top servo is positioned in open top cover position, from close position
                time.sleep(t_flip_open_time)         # time for the top servo to reach the open top cover position
                t_top_cover='open'                   # variable to track the top cover/lifter position
                b_servo_operable=True                # variable to block/allow bottom servo operation
        
    if b_servo_home==False:                          # case bottom servo is not home
        if not stop_servos:                          # case there is not a stop request for servos
            if b_servo_operable==True:               # variable to block/allow bottom servo operation
                if b_servo_CW_pos==True:             # boolean of bottom servo at full CW position
                    b_servo.value = b_home_from_CW   # bottom servo moves to the extra home position, from CCW
            
                elif b_servo_CCW_pos==True:          # boolean of bottom servo at full CCW position
                    b_servo.value = b_home_from_CCW  # bottom servo moves to the extra home position, from CW
                
                time.sleep(b_spin_time)              # time for the bottom servo to reach the extra home position
                b_servo_home=True                    # boolean bottom servo is home

    
    time.sleep(0.25)                                 # little delay, to timely separate from previous robot movements
    runs=8                                           # number of sections
    b_delta = (b_servo_CW-b_home)/runs               # PWM amplitute per section
    t_delta = 1.3*b_spin_time/runs                   # time amplitude per section
    k=1                                              # coefficient that treats differently the first rotation, as it start from home
    
    for i in range(4,runs):                          # iteration over the sections, starting somehome from the middle
        b_target_CCW=b_home+b_delta*(runs-i)         # PWM target calculation for CCW postion
        delay_time=t_delta*(runs-i)                  # time calculation for the servo movement
        if not stop_servos:                          # case there is not a stop request for servos
            b_servo_stopped=False                    # boolean of bottom servo at location the lifter can be operated
            b_servo.value = b_target_CCW             # bottom servo moves to the target_CCW position
            time.sleep(k*(delay_time+i*0.01))        # time for the bottom servo to reach the target position
            k=2                                      # coefficient to double the time, at each move do not start from home anymore
            b_target_CW=b_home-b_delta*(runs-i)      # PWM target calculation for CW postion
            delay_time=t_delta*(runs-i)              # time calculation for the servo movement
        if not stop_servos:                          # case there is not a stop request for servos
            b_servo_operable=False                   # variable to block/allow bottom servo operation
            b_servo.value = b_target_CW              # bottom servo moves to the target_CW position
            time.sleep(k*(delay_time+i*0.01))        # time for the bottom servo to reach the target position
            b_servo_stopped=True                     # boolean of bottom servo at location the lifter can be operated
    
    if not stop_servos:                              # case there is not a stop request for servos
        b_servo_stopped=False                        # boolean of bottom servo at location the lifter can be operated
        b_servo.value = b_home                       # bottom servo moves to home position
        time.sleep(k*(delay_time+i*0.010))           # time for the bottom servo to reach home position
        b_servo_stopped=True                         # boolean of bottom servo at location the lifter can be operated
        b_servo_home=True                            # boolean bottom servo is home

    stopping_servos(print_out=s_debug)
    
    return True







def servo_solve_cube(moves, print_out=s_debug):
    """ Function that translates the received string of moves, into servos sequence activations.
        This is substantially the main function."""
    
    global t_top_cover, b_servo_operable, b_servo_stopped, b_servo_home
    
    start_time=time.time()                         # start time is assigned
    
    # the received string is analyzed if compatible with servo rotation contraints, and amount of movements
    servo_angle_ok, tot_moves, remaining_moves = check_moves(moves, print_out=s_debug)
    
    if not servo_angle_ok:
        print("Error on servo moves algorithm")    # feedback to terminal
        robot_status_='Robot_stopped'              # string variable indicating how the servo_solve_cube function has ended
        robot_time_=(time.time()-start_time)       # robot time is calculated
        return robot_status_, robot_time_          # function returns the cube status and the robot time
        
    if print_out:
        print(f'total amount of servo movements: {tot_moves}\n')    
    
    string_len=len(moves)                          # number of characters in the moves string
    for i in range(string_len):                    # iteration over the characters of the moves string
        if stop_servos:                            # case there is a stop request for servos
            break                                  # the foor loop in interrupted
        
        if i%2==0:
            display_progress_bar(remaining_moves[i])
        
        if moves[i]=='F':                          # case there is a flip on the move string
            flips=int(moves[i+1])                  # number of flips
            if print_out:
                print(f'To do F{flips}')           # for print_out
            
            for f in range(flips):                 # iterates over the number of requested flips        
                if stop_servos:                    # case there is a stop request for servos
                    break                          # the foor loop in interrupted
                
                flip_up()                          # lifter is operated to flip the cube

                if f<(flips-1):                    # case there are further flippings to do
                    flip_to_open()                 # lifter is lowered stopping the top cover in open position (cube not constrained)

# alternative choice
#                     flip_to_close()   # lifter is lowered stopping the top cover in close position (cube constrained, for better facelets alignment)
                
                if f==(flips-1) and string_len-(i+2)>0:   # case it's the last flip and there is a following command on the move string
                    if moves[i+2]=='R':            # case the next action is a 1st layer cube rotation
                        flip_to_close()            # top cover is lowered to close position
                    elif moves[i+2]=='S':          # case the next action is a cube spin
                        flip_to_open()             # top cover is lowered to open position
        


        elif moves[i]=='S':                        # case there is a cube spin on the move string
            direction=int(moves[i+1])              # rotation direction is retrived
            if print_out:
                print(f'To do S{direction}')       # for print_out

            if direction==3:                       # case the direction is CCW
                set_dir='CCW'                      # CCW directio is assigned to the variable
            else:                                  # case the direction is CW
                set_dir='CW'                       # CW directio is assigned to the variable
            
            if b_servo_home==True:                 # case bottom servo is at home
                spin_out(set_dir)                  # call to function to spin the full cube to full CW or CCW
            
            else:                                  # case the bottom servo is at full CW or CCW position
                spin_home()                        # call to function to spin the full cube toward home position


        elif moves[i]=='R':                        # case there is a cube 1st layer rotation
            direction=int(moves[i+1])              # rotation direction is retrived   
            if print_out:
                print(f'To do R{direction}')       # for print_out

            if direction==3:                       # case the direction is CCW
                set_dir='CCW'                      # CCW directio is assigned to the variable
            else:                                  # case the direction is CW
                set_dir='CW'                       # CW directio is assigned to the variable
            
            if b_servo_home==True:                 # case bottom servo is at home
                rotate_out(set_dir)                # call to function to rotate cube 1st layer on the set direction, moving out from home              
            
            elif b_servo_CW_pos==True:             # case the bottom servo is at full CW position
                if set_dir=='CCW':                 # case the set direction is CCW
                    rotate_home(set_dir)           # call to function to spin the full cube toward home position
                
            elif b_servo_CCW_pos==True:            # case the bottom servo is at full CCW position
                if set_dir=='CW':                  # case the set direction is CW
                    rotate_home(set_dir)           # call to function to spin the full cube toward home position
    
    if stop_servos:                                # case there is a stop request for servos 
        if print_out:
            print("\nRobot stopped")
        robot_status_='Robot_stopped'              # string variable indicating how the servo_solve_cube function has ended
        stopping_servos(s_debug)                     # call the stop servo function
        
    elif not stop_servos:                          # case there is not a stop request for servos
        robot_status_='Cube_solved'                # string variable indicating how the servo_solve_cube function has ended
        
        if print_out:
            if tot_moves!=0:
                print("\nCompleted all the servo movements")
            else:
                print("\nno servo movements needed")

    robot_time_=(time.time()-start_time)           # robot time is calculated
    
    return robot_status_, robot_time_              # function returns the cube status and the robot time



     
     




if __name__ == "__main__":
    """ This main function (without arguments) to test the servos while solving a predefined robot movement string:
        F2R1S3R1S3S3F1R1F2R1S3S3F1R1S3R1F3R1S3R1S3S3F3R1S3F1R1S3R1F3R1S3R1S3F3R1S3R1
        F1R3S1F1R3F1S1R3S3F1R1S3R1F3S1R3F1R1S3S3F3R1S3R1F3R1S3R1S3F1S1R3S1F3R3F1R1S3'
        
        When argument 'set' from the command line, this function allows to easily set the serco the mid position,
        by passing 0 (zero); Other angles can be tested, by passing values from -1 to 1."""    
    
    
    ##################### setting the servo to mid angle, or to traget, from the CLI  #############
    import argparse
    parser = argparse.ArgumentParser(description='Find out the servo parameter for middle position')            # argument parser object creation
    parser.add_argument("--set", type=float, help="Set servos to PWM value (value range from -1.00 to 1.00)")   # argument is added to the parser
    args = parser.parse_args()                                              # argument parsed assignement
    
    
    def set_servo(target):
        """ Function to set the servos to target angle, wherein the target is from -1.00 to 1.00"""
        
        target=round(float(target),2)                                      # target argument is rounded to two decimals
        if target < -1:                                                    # case the received target is smaller than -1
            target = -1.0                                                  # target is set to -1
            print('accepted values are from -1.00 to 1.00: used -1.00')    # feedback is printed to the terminal
        elif target > 1:                                                   # case the received target is bigger than 1
            target = 1.0                                                   # target is set to 1
            print('accepted values are from -1.00 to 1.00: used 1.00')     # feedback is printed to the terminal
        elif target == 0:                                                  # case the received target equals zero
            print('servo to: MID POSITION')                                # feedback is printed to the terminal
        else:                                                              # case the received target is between -1 and 1 included, but not zero
            print('servo to:', target)                                     # feedback is printed to the terminal
        t_servo.value = target                                             # top servo is set to the target
        b_servo.value = target                                             # bottom servo is set to the target
    
    
    
    if args.set != None:                                   # case the Cubotino_T_servo.py has been launched with 'set' argument
        t_servo = Servo(t_servo_pin, pin_factory=factory)  # top servo object is created, by using HD based timers for the PWM
        b_servo = Servo(b_servo_pin, pin_factory=factory)  # bottom servo object is created, by using HD based timers for the PWM
        set_servo(args.set)                                # servos are set according to the value in 'set' argument
        while True:                                        # infinite loop, to give the chance to play with the servos angles
            target = input('\nenter a new PWM value from -1.00 to 1.00 (0 for mid position, any letter to escape): ') # input request with proper info
            try:                                           # tentative 
                target = float(target)                     # user input transformed to float
                set_servo(target)                          # servos are set according to the input value
            except:                                        # exception, in case non-numbers entered
                print('\nquit')                            # feedback is printed to the terminal 
                break                                      # while loop is interrupted
    # #############################################################################################
         
        
           
    
    ###############  testing the servos    ########################################################

    else:     # case the Cubotino_T_servo.py has been launched without 'set' argument
        
        # set demo to True to let the robot to manouvre the cube, to verify if the servos angles and timers are OK
        # set demo to False to manually test the servos positions, by typing on the REPL:
        # t_servo.value = t_servo_close
        # t_servo.value = t_servo_open
        # t_servo.value = t_servo_read
        # t_servo.value = t_flip_up
        # b_servo.value = b_home
        # b_servo.value = b_servo_CCW
        # b_servo.value = b_servo_CW

        demo = True   # this boolean to test one servo pos at the time (set False), or all together (set True)
        
        if not demo:    # case demo is set false
            init_servo(print_out=s_debug)      # servos are initialized
            print('\nenter manually the command at REPL, i.e. t_servo.value = t_servo_close')
        
        elif demo:    # case demo is set true
            print('\ndemonstration of the robot servos current settings')
            movements='F2R1S3R1S3S3F1R1F2R1S3S3F1R1S3R1F3R1S3R1S3S3F3R1S3F1R1S3R1F3R1S3R1S3F3R1S3R1F1R3S1F1R3F1S1R3S3F1R1S3R1F3S1R3F1R1S3S3F3R1S3R1F3R1S3R1S3F1S1R3S1F3R3F1R1S3'
            # the complete the moves of this string CUBOTino takes 1m:8secs (18/04/2022)
            
            s_disp.set_backlight(1)              # activates the display backlight
            s_debug= False  # True               # boolean for s_debug printouts purpose
            
            robot_init_status = False            # False to prevent servos to be positioned as per robot start
            if init_servo(print_out=s_debug):    # servos are initialized
                print('servo init done\n')       # print feedback
                robot_status, robot_time = servo_solve_cube(movements, print_out=s_debug)   # robot solver is called

                if robot_status == 'Cube_solved':                                # case the robot solver returns Cube_solved in the robot_status
                    print("\nCube is solved")                                    # print the status as feedback
                    print(f"Solving time: {round(robot_time,1)} secs")           # print the solving time as feedback
                    fun(print_out=s_debug)                                         # fun function ... some cube_holder movements to get attention
                    
                elif robot_status == 'Robot_stopped':                            # case the robot solver returns Robot_stopped in the robot_status
                    print(f"\nRobot has been stopped, after {round(robot_time,1)} secs")  # print the status as feedback

            s_disp.set_backlight(0)                # de-activates the display backlight
    # #############################################################################################

