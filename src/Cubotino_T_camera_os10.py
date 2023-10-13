#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
#  Andrea Favero 12 October 2023
#
# This script relates to CUBOTino autonomous, a very small and simple Rubik's cube solver robot 3D printed
# CUBOTino autonomous is the 'Top version', of the CUBOTino robot series.
# This specific script manages the camera, for Raspberry Pi OS ==10
# This file is imported by Cubotino_T.py
#
#############################################################################################################
"""

from picamera.array import PiRGBArray        # Raspberry Pi specific package for the camera, using numpy array
from picamera import PiCamera                # Raspberry Pi specific package for the camera
import os.path, pathlib, json                # library for the json parameter parsing for the display
from getmac import get_mac_address           # library to get the device MAC ddress
from get_macs_AF import get_macs_AF          # import the get_macs_AF function
macs_AF = get_macs_AF()                      # mac addresses of AF bots are retrieved
import time
import sys


class Camera:
    
    def __init__(self):
        """ Imports and set the picamera (V1.3)"""
        print("\nLoading camera parameters")
        
        # convenient choice for Andrea Favero, to upload the settings fitting my robot, via mac check
        fname = 'Cubotino_T_settings.txt'                             # fname for the text file to retrieve settings
        folder = pathlib.Path().resolve()                             # active folder (should be home/pi/cube)  
        eth_mac = get_mac_address()                                   # mac address is retrieved
        if eth_mac in macs_AF:                                        # case the script is running on AF (Andrea Favero) robot
            pos = macs_AF.index(eth_mac)                              # return the mac addreess position in the tupple
            fname = self.get_fname_AF(fname, pos)                     # generates the AF filename
        else:                                                         # case the script is not running on AF (Andrea Favero) robot
            fname = os.path.join(folder, fname)                       # folder and file name for the settings, to be tuned
        if os.path.exists(fname):                                     # case the settings file exists
            with open(fname, "r") as f:                               # settings file is opened in reading mode
                settings = json.load(f)                               # json file is parsed to a local dict variable
            try:                                                      # tentative
                camera_width_res = int(settings['camera_width_res'])  # Picamera resolution on width 
                camera_height_res = int(settings['camera_hight_res']) # Picamera resolution on heigh
                s_mode = int(settings['s_mode'])                      # camera setting mode (pixels binning)
                self.kl = float(settings['kl'])                       # coff. for PiCamera stability acceptance

            except:   # exception will be raised if json keys differs, or parameters cannot be converted to int/float
                print('error on converting the imported parameters to int, float or string')   # feedback is printed to the terminal
                
        else:                                                         # case the settings file does not exists, or name differs
            print(f'could not find {fname}')                          # feedback is printed to the terminal
            
        print("Setting camera\n")                                     # feedback is printed to the terminal
        self.width = camera_width_res                                 # image width
        self.height = camera_height_res                               # image height
        
        # PiCamera V1.3: sensor mode 7 means 4x4 binning, 4:3, Full Field of View
        # PiCamera V2:   sensor mode 4 means 2x2 binning, 4:3, Full Field of View
        self.cam = PiCamera(sensor_mode=s_mode)                       # sets the camera mode (resolution and binnig)
        self.rawCapture = PiRGBArray(self.cam)                        # returns (uncoded) RGB array from the camera
        self.cam.resolution = (self.width, self.height)               # camera.resolution is an attribute, not a method 

    
    def get_width(self):
        return self.cam.resolution[0]
    
    def get_height(self):
        return self.cam.resolution[1]
    
    def get_binning(self):
        return self.cam.sensor_mode
        
    def get_frame(self):
        self.cam.capture(self.rawCapture, format='bgr')  # bgr is the picamera format compatible with CV2
        self.frame = self.rawCapture.array               # picamera array allows usage of numpy array
        self.rawCapture.truncate(0)                      # empties the array in between each camera's capture
        return self.frame
    
    def get_fname_AF(self, fname, pos):
        return fname[:-4] + '_AF' + str(pos+1) + '.txt'
    
    def printout(self):
        a = '\nPicamera defined for os_version 10'
        b = f'\nPiCamera resolution (width x height): {self.cam.resolution}'  # feedback is printed to the terminal
        self.binn = self.cam.sensor_mode                 # PiCamera sensor_mode is checked
        #  sensor_mode answer from the camera is interpreted
        if self.binn <=3:                                # case the binning is <=3 it means no binning
            binning='none'                               # binning variable is set to none
        elif self.binn == 4 or self.binn == 5:           # case the binning is 4 or 5 it means binning 2x2
            binning='2x2'                                # binning variable is set to 2x2
        elif self.binn > 5:                              # case binning is bigger than 5 it means binning 4x4
            binning='Binning is 4x4 if PiCamera V1.3, differently 2x2 if V2'  # binning 4x4 when PiCamera V1.3, 2x2 when PiCamera V2
        c = f'\nPiCamera mode (binning): {binning}'      # feedback is printed to the terminal
        return a + b + c
        
    def close_camera(self):
        self.cam.close()
        return 'camera closed'
    
    def is_stable(self, debug):
        """ According to PiCamera documentation it is required a couple of seconds to warmup the camera.
            This fuction releases the camera warm-up phase only after all the gains are stable,
            meaning an absolute variation within set limits (< 3%) from the average of last 2 seconds."""
        
        if debug:                              # case debug variable is set true on __main__
            print('camera set in auto mode')   # feedback is printed to the terminal
        self.cam.exposure_mode = 'auto'        # set to auto exposure at the start, to adjust according to light conditions
        time.sleep(0.05)              # not found documentation if a delay is needed after this PiCamera setting 
        self.cam.awb_mode = 'auto'    # set to auto white balance at the start, to adjust according to light conditions
        time.sleep(0.05)              # not found documentation if a delay is needed after this PiCamera setting
        self.cam.shutter_speed = 0    # set to shutter speed to auto at the start, to adjust according to light conditions
        time.sleep(0.05)              # not found documentation if a delay is needed after this PiCamera setting
        
        a_gain_list=[]                # list to store the Picamera analog gain, during warmup period
        d_gain_list=[]                # list to store the Picamera digital gain, during warmup period
        awb_blue_list=[]              # list to store the Picamera AWB gain, for blue, during warmup period
        awb_red_list=[]               # list to store the Picamera AWB gain, for red, during warmup period
        exp_list=[]                   # list to store the Picamera exposure time, during warmup period
        t_list=[]                     # list to store the warmup progressive time of checkings
        PiCamera_param=()             # empty tuple is assigned to PiCamera_param variable
        
        kl = self.kl                  # lower koefficient to define acceptance bandwidth (i.e. 0.95 == 95%)
        ku = 2-kl                     # Upper koefficient to define acceptance bandwidth (105%)
        pts = 8                       # amount of consecutive datapoints to analyse if parameters within acceptable range
        
        t_start=time.time()           # time reference is assigned as reference for the next task
        
        while time.time()-t_start <20:  # timeout for camera warm-up phase is 20 seconds
#             frame = self.get_frame()                       # camera start reading the cube, and adjusts the awb/exposure
            a_gain=self.cam.analog_gain                    # analog gain is inquired to the PiCamera
            d_gain=self.cam.digital_gain                   # digital gain is inquired to the PiCamera
            awb_gains=self.cam.awb_gains                   # awb blue and red gains are inquired to the PiCamera
            exposure=self.cam.exposure_speed               # exposure is inquired to the PiCamera
            
            a_gain_list.append(round(float(a_gain),2))     # analog gain is appended to a list
            d_gain_list.append(round(float(d_gain),2))     # digital gain is appended to a list
            awb_blue_list.append(round(float(awb_gains[0]),2))  # awb blue part gain is appended to a list
            awb_red_list.append(round(float(awb_gains[1]),2))   # awb blue part gain is appended to a list
            exp_list.append(exposure)                      # exposure time (micro secs) is appended to a list
            t_list.append(round(time.time()- t_start,2))   # time (from AWB and Exposure start adjustement) is appended to a list
            
            a_check=False                                  # a flag is negatively set for analog gain (being stable...)
            d_check=False                                  # a flag is negatively set for digital gain (being stable...)
            awb_blue_check=False                           # a flag is negatively set for awb blue gain (being stable...)
            awb_red_check=False                            # a flag is negatively set for awb red gain (being stable...)
            exp_check=False                                # a flag is negatively set for the exposure time (being stable...)
            time.sleep(0.15)
            
            if len(a_gain_list) > pts:                     # requested a minimum amount of datapoints
                check = a_gain_list[-1]/(sum(a_gain_list[-pts:])/pts)  # last analog gain value is compared to the average of last pts points
                if check > kl and check < ku:              # if comparison within acceptance boundaries
                    a_check=True                           # flag is positively set for this gain

                check = d_gain_list[-1]/(sum(d_gain_list[-pts:])/pts)  # last digital gain value is compared to the average of last pts points
                if check > kl and check < ku:              # if comparison within acceptance boundaries
                    d_check=True                           # flag is positively set for this gain               
            
                check = awb_blue_list[-1]/(sum(awb_blue_list[-pts:])/pts) # last awb_blue gain is compared to the average of last pts points
                if check > kl and check < ku:              # if comparison within acceptance boundaries
                    awb_blue_check=True                    # flag is positively set for this gain    
                    
                check = awb_red_list[-1]/(sum(awb_red_list[-pts:])/pts) # last awb_red gain is compared to the average of last pts points
                if check > kl and check < ku:              # if comparison within acceptance boundarie
                    awb_red_check=True                     # flag is positively set for this gain
                
                check = exp_list[-1]/(sum(exp_list[-pts:])/pts)  # last exposure time is compared to the average of last pts points
                if check > kl and check < ku:              # if comparison within acceptance boundarie
                    exp_check=True                         # flag is positively set for the exposure time                
         
                if a_check and d_check and awb_blue_check and awb_red_check and exp_check: # if all gains are stable
                    PiCamera_param=(a_gain, d_gain, awb_gains, exposure)  # latest parameters returned by the PiCamera are assigned to a tuple
                    break                                  # camera warmup while loop break
        
        if debug:
            print('\nPiCamera warmup function:')      # feedback is printed to the terminal
            print('analog_gain_list',a_gain_list)     # feedback is printed to the terminal
            print('digital_gain_list',d_gain_list)    # feedback is printed to the terminal
            print('awb_blue_list',awb_blue_list)      # feedback is printed to the terminal
            print('awb_red_list',awb_red_list)        # feedback is printed to the terminal
            print('camera exp_list', exp_list)        # feedback is printed to the terminal
            print('time:', t_list)                    # feedback is printed to the terminal
            print('datapoints:', len(t_list))         # feedback is printed to the terminal
        
        print(f'PiCamera: AWB and Exposure being stable in {round(t_list[-1],1)} secs')
        return PiCamera_param

    
    def set_gains(self, debug, a_gain, d_gain, awb_gains):
        import Cubotino_T_set_picamera_gain as camera_set_gains  # script that allows to fix some parameters at picamera
        if debug:                                     # case debug variable is set true
            print('camera awb set off')               # feedback is printed to the terminal
        self.cam.awb_mode = 'off'                     # sets white balance off
        time.sleep(0.05)                              # small (arbitrary) delay after setting a new parameter to PiCamera
        self.cam.awb_gains = awb_gains                # sets AWB gain to PiCamera, for consinsent images 
        time.sleep(0.05)                              # small (arbitrary) delay after setting a new parameter to PiCamera
        camera_set_gains.set_analog_gain(self.cam, a_gain)    # sets analog gain to PiCamera, for consinsent images
        time.sleep(0.05)                              # small (arbitrary) delay after setting a new parameter to PiCamera
        camera_set_gains.set_digital_gain(self.cam, d_gain)   # sets digital gain to PiCamera, for consinsent images
        time.sleep(0.05)                              # small (arbitrary) delay after setting a new parameter to PiCamera
        self.cam.shutter_speed = 0                    # set the shutter speed to auto at the start, to adjust according to light conditions
        time.sleep(0.05)                              # small (arbitrary) delay after setting a new parameter to PiCamera


    def get_exposure(self):
        return self.cam.exposure_speed
    
    
    def set_exposure(self, shutter_time):
        self.cam.shutter_speed = shutter_time         # sets the shutter time to the PiCamera, for consistent images
        time.sleep(0.05)                              # small (arbitrary) delay after setting a new parameter to PiCamera 
        





camera = Camera()

# if __name__ == "__main__":
#     """the main function can be used to test the camera. """
# 
# 
#     print(camera.binning)
#     print(camera.cam.resolution)
#     print(camera.debug)

