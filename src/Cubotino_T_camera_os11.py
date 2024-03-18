#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
#  Andrea Favero 10 March 2024
#
# This script relates to CUBOTino Pocket, a very small and simple Rubik's cube solver robot 3D printed
# CUBOTino autonomous is the CUBOTino versions for the Rubik's cube 2x2x2.
# This specific script manages the camera, for Raspberry Pi OS 11 (Bullseye).
# This file is imported by Cubotino_T.py and Cubotino_servos_GUI.py.
#
#############################################################################################################
"""

from Cubotino_T_settings_manager import settings as settings  # settings manager Class
from picamera2 import Picamera2        # Raspberry Pi specific package for the camera, since Raspberry Pi OS 11
from libcamera import controls
import time


class Camera:
    
    def __init__(self):
        """ Imports and set the picamera (V1.3)"""
        
        print("\nLoading camera parameters")
        sett = settings.get_settings()                   # settings are retrieved from the settings Class
        camera_width_res = sett['camera_width_res']      # Picamera resolution on width 
        camera_height_res = sett['camera_hight_res']     # Picamera resolution on heigh
        self.kl = sett['kl']                             # coff. for PiCamera stability acceptance
        self.expo_shift = sett['expo_shift']             # Picamera shift on exposure value
  
            
        print("Setting up the camera\n")                              # feedback is printed to the terminal
        self.width = camera_width_res                                 # image width
        self.height = camera_height_res                               # image height
        
        self.cam = Picamera2()                                        # camera object is defined
        
        # configguration object for the preview stream
        self.config = self.cam.create_preview_configuration({"size": (self.width, self.height), "format":"RGB888"})
        self.cam.preview_configuration.align(self.config)             # additional setting, to align (round) resolutions to the convenient ones
        self.cam.configure(self.config)                               # configuration is applied to the camera
        time.sleep(1)
        self.cam.set_controls({"ExposureValue": self.expo_shift})     # exposition target is shifted by expo_shift value (range from -8 to 8)
        self.cam.start()                                              # camera (object) is started
        print()                                                       # an empty line is printed for separation
         
    
    def set_resolution(self, w, h):
        self.cam.stop()
        self.config = self.cam.create_preview_configuration({"size": (w, h), "format":"RGB888"})
        self.cam.preview_configuration.align(self.config)
        self.cam.configure(self.config)
        self.cam.start()
        time.sleep(0.15)                # little delay to let the camera setting
    
    
    def get_width(self):
        return self.config['main']['size'][0]
    
    
    def get_height(self):
        return self.config['main']['size'][1]
    
    
    def get_frame(self):
        for i in range(3):
            frame = self.cam.capture_array()
        return frame
    
    
    def get_fname_AF(self, fname, pos):
        return fname[:-4] + '_AF' + str(pos+1) + '.txt'
    
    
    def printout(self):
        a = '\nPicamera defined for os_version 11'
        b = f'\nPiCamera resolution (width, height): {self.config["main"]["size"]}'  # feedback is printed to the terminal
        return a + b
    
    
    def close_camera(self):
        self.cam.close()
        return 'camera closed'
    
    
    def set_auto(self, debug, awb_mode,expo_shift):
        self.cam.stop()
        
        # set to auto exposure and white balance at the start, to adjust according to light conditions        
        with self.cam.controls as controls:
            
            controls.AeEnable = True
            controls.AwbEnable = True
            
            # AeExposureMode: 0=Normal - normal exposures, 1=Short - use shorter exposures, 2=Long - use longer exposures, 3=Custom - use custom exposures
            controls.AeExposureMode = 1
            
            # AwbMode: 0=Auto - any illumant, 1=Tungsten - tungsten lighting, 2=Fluorescent - fluorescent lighting
            # 3=Indoor - indoor illumination, 4=Daylight - daylight illumination, 5=Cloudy - cloudy illumination, 6=Custom - custom setting
            controls.AwbMode = awb_mode
            
            # AeMeteringModeEnum: 0=CentreWeighted - centre weighted metering, 1=Spot - spot metering, 2=Matrix - matrix metering, 3=Custom - custom metering
            controls.AeMeteringMode = 0
            
        time.sleep(0.2)
        self.cam.set_controls({"ExposureValue": expo_shift})     # exposition target is shifted by expo_shift value (range from -8 to 8)

        self.cam.start()
        if debug:                              # case debug variable is set true on __main__
            # feedback is printed to the terminal
            print('\nCamera set in automatic mode, with AwbMode type:', awb_mode, "  and expo_shift:", expo_shift)
    
    
    
    def set_gains(self, debug, a_gain, d_gain, awb_gains):
        # d_gain is not used (maintained for compatibiity with Cubotino_T_camera_os10 version)
        self.cam.stop()
        self.cam.set_controls({"AnalogueGain":a_gain, "ColourGains":awb_gains})
        self.cam.start()
        time.sleep(0.1)                               # small (arbitrary) delay after setting a new parameter to PiCamera
        
    
    def get_camera_controls(self):
        return self.cam.camera_controls

    
    def get_metadata(self):
        return self.cam.capture_metadata()             # image metadata is inquired
    
    
    def get_exposure(self):
        metadata = self.cam.capture_metadata()        # image metadata is inquired
        return  metadata["ExposureTime"]              # exposure time from metadata is assigned to the variable 
    
    
    def set_exposure(self, shutter_time):
        self.cam.stop()
        self.cam.set_controls({"ExposureTime":shutter_time}) # sets the shutter time to the PiCamera, for consistent images
        self.cam.start()
        time.sleep(0.05)                              # small (arbitrary) delay after setting a new parameter to PiCamera 
    
    
    def set_exposure_shift(self, shift):
        self.cam.stop()
        shift = float(shift)
        self.cam.set_controls({"ExposureValue":shift}) # sets a shift (from -8 to +8) on the auto exposure
        self.cam.start()
        time.sleep(0.05)                              # small (arbitrary) delay after setting a new parameter to PiCamera 







camera = Camera()

if __name__ == "__main__":
    """the main function can be used to test the camera. """

    print(camera.printout())



