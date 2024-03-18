#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
#  Andrea Favero 10 March 2023
#
# This script relates to CUBOTino autonomous, a very small and simple Rubik's cube solver robot 3D printed
# CUBOTino autonomous is the 'Top version', of the CUBOTino robot series.
# This specific script manages the camera, for Raspberry Pi OS ==10 (Buster).
# This file is imported by Cubotino_T.py and Cubotino_servos_GUI.py.
#
#############################################################################################################
"""

from Cubotino_T_settings_manager import settings as settings  # settings manager Class
from picamera.array import PiRGBArray        # Raspberry Pi specific package for the camera, using numpy array
from picamera import PiCamera                # Raspberry Pi specific package for the camera
import time


class Camera:
    
    def __init__(self):
        """ Imports and set the picamera (V1.3)"""
        
        print("\nLoading camera parameters")             # feedback is printed to the terminal
        sett = settings.get_settings()                   # settings are retrieved from the settings Class
        
        camera_width_res = sett['camera_width_res']      # Picamera resolution on width 
        camera_height_res = sett['camera_hight_res']     # Picamera resolution on heigh
        s_mode = sett['s_mode']                          # camera setting mode (pixels binning)
        self.kl = sett['kl']                             # coff. for PiCamera stability acceptance
            
        print("Setting camera\n")                        # feedback is printed to the terminal
        self.width = camera_width_res                    # image width
        self.height = camera_height_res                  # image height
        
        # PiCamera V1.3: sensor mode 7 means 4x4 binning, 4:3, Full Field of View
        # PiCamera V2:   sensor mode 4 means 2x2 binning, 4:3, Full Field of View
        self.cam = PiCamera(sensor_mode=s_mode)          # sets the camera mode (resolution and binnig)
        self.rawCapture = PiRGBArray(self.cam)           # returns (uncoded) RGB array from the camera
        self.cam.resolution = (self.width, self.height)  # camera.resolution is an attribute, not a method 

    
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
    

    def set_auto(self, debug):
        self.cam.exposure_mode = 'auto'  # set to auto exposure at the start, to adjust according to light conditions
        time.sleep(0.05)              # not found documentation if a delay is needed after this PiCamera setting 
        self.cam.awb_mode = 'auto'    # set to auto white balance at the start, to adjust according to light conditions
        time.sleep(0.05)              # not found documentation if a delay is needed after this PiCamera setting
        self.cam.shutter_speed = 0    # set to shutter speed to auto at the start, to adjust according to light conditions
        time.sleep(0.05)              # not found documentation if a delay is needed after this PiCamera setting
        if debug:                     # case debug variable is set true on __main__
            print('Camera set in auto mode')   # feedback is printed to the terminal
        

    def get_metadata(self):
        a_gain=self.cam.analog_gain                   # analog gain is inquired to the PiCamera
        d_gain=self.cam.digital_gain                  # digital gain is inquired to the PiCamera
        awb_gains=self.cam.awb_gains                  # awb blue and red gains are inquired to the PiCamera
        exposure=self.cam.exposure_speed              # exposure is inquired to the PiCamera
        return a_gain, d_gain, awb_gains, exposure    # fains and exposure are returned
            
            
    def set_gains(self, debug, a_gain, d_gain, awb_gains):
        import Cubotino_T_set_picamera_gain as camera_set_gains  # script that allows to fix some parameters at picamera
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
        if debug:                                     # case debug variable is set true
            print('Camera set in manual mode')        # feedback is printed to the terminal



    def get_exposure(self):
        return self.cam.exposure_speed
    
    
    def set_exposure(self, shutter_time):
        self.cam.shutter_speed = shutter_time         # sets the shutter time to the PiCamera, for consistent images
        time.sleep(0.05)                              # small (arbitrary) delay after setting a new parameter to PiCamera 
        





camera = Camera()

if __name__ == "__main__":
    """the main function can be used to test the camera. """


    print(camera.binning)
    print(camera.cam.resolution)
    print(camera.debug)

