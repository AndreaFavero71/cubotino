#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
#  Andrea Favero 10 March 2024
#
# This script relates to CUBOTino Autonomous, a small and simple Rubik's cube solver robot 3D printed
# This specific script manages the interactions with the settings files (json text files).
#
# Parameters at the settings files are those more likely to vary between differentt robots.
# Having the setting "insulated" in text files makes easier to list, document, change them.
#
# Mac address is used to match boards and settings on my robots: This limits mistakes at my end. 
#
#############################################################################################################
"""

import sys, glob, os.path, pathlib, json
from getmac import get_mac_address           # library to get the device MAC ddress
from get_macs_AF import get_macs_AF          # import the get_macs_AF function


class Settings:

    def __init__(self):
        """Check if local settings exist, if not are created.
            Reads the local settings files.
            Parameters and settings are imported.
            Settings datatypes are parsed."""
        
        # mac address is used by myself (Andrea Favero), to upload the settings fitting my robot
        self.macs_AF = get_macs_AF()                      # mac addresses of AF bots are retrieved
        self.folder = pathlib.Path().resolve()            # active folder (should be home/pi/cubotino/src)  
        self.eth_mac = get_mac_address().lower()          # mac address is retrieved
        
        self.check_local_settings()                       # checks whether local settings files exist
        
        s = self.read_settings()                          # reads the general robot settings
        servos_s = self.read_servos_settings()            # reads the servos settings
        
        self.s = self.parse_settings(s)                   # converts settings to correct datatype
        self.servos_s = self.parse_servos_settings(servos_s) # converts servos settings to correct datatype





    def get_fname_AF(self, fname, pos):
        """Generate settings filenames for AF robot."""
        return fname[:-4] + '_AF' + str(pos+1) + '.txt'




    def check_local_settings(self):
        """Check the presence of local settings, If not present, the default settings files
            are copied as local settings files."""
        
        fnames_default = ('Cubotino_T_settings_default.txt' , 'Cubotino_T_servo_settings_default.txt')
        fnames_local = ('Cubotino_T_settings.txt', 'Cubotino_T_servo_settings.txt')
        
        for i in range(2):
            fname = fnames_local[i]                               # file name to locally save general settings
            
            if self.eth_mac in self.macs_AF:                      # case the script is running on AF (Andrea Favero) robot
                pos = self.macs_AF.index(self.eth_mac)            # mac address row position in macs_AF.txt file
                fname = self.get_fname_AF(fname, pos)             # AF robot settings file name
            else:                                                 # case the script is not running on AF (Andrea Favero) robot
                fname = os.path.join(self.folder,fname)           # folder and file name for the settings, to be tuned
            
            if not os.path.exists(fname):                         # case the local settings file does not exist
                fname_d = os.path.join(self.folder,fnames_default[i]) # folder and file name for the settings, to be tuned
                
                if not os.path.exists(fname_d):                   # case the default settings file does not exist
                    print("\n\n\n")                               # print empty lines
                    print("#"*86)                                 # print a separation line
                    print(f"Missed file {fnames_default[0]}  or  {fnames_default[1]}") # feedback is printed to the terminal
                    print("Double check the wastebacket if these files were erared by mistake.") # feedback is printed to the terminal
                    print("These files are anyhow available at GitHub project page") # feedback is printed to the terminal
                    print("#"*86, "\n\n\n")                       # print a separation line
                    sys.exit(1)                                   # script is quitted with error
                
                with open(fname_d, 'r') as input:                 # default settings file is opened as input
                    output = open(fname, 'w')                     # local settings file is opened to be written
                    output.write(input.read())                    # default settings are saved into the local settings file
                
                print(f"One time action: Creating local settings file {fname}")  # feedback is printed to the terminal





    def get_settings(self):
        """Returns the general robot settings dict."""
        return self.s





    def get_servos_settings(self):
        """Returns the servos settings dict."""
        return self.servos_s





    def get_settings_fname(self):
        fname = 'Cubotino_T_settings.txt'                         # file name with general settings
        if self.eth_mac in self.macs_AF:                          # case the script is running on AF (Andrea Favero) robot
            pos = self.macs_AF.index(self.eth_mac)                # mac address row position in macs_AF.txt file
            fname = self.get_fname_AF(fname, pos)                 # AF robot s file name (do not use these at the start)
        else:                                                     # case the script is not running on AF (Andrea Favero) robot
            fname = os.path.join(self.folder,fname)               # folder and file name for the settings, to be tuned
        return fname                                              # return folder and file name for the settings





    def get_servo_settings_fname(self):
        fname = 'Cubotino_T_servo_settings.txt'                   # file name with servos settings
        if self.eth_mac in self.macs_AF:                          # case the script is running on AF (Andrea Favero) robot
            pos = self.macs_AF.index(self.eth_mac)                # mac address row position in macs_AF.txt file
            fname = self.get_fname_AF(fname, pos)                 # AF robot s file name (do not use these at the start)
        else:                                                     # case the script is not running on AF (Andrea Favero) robot
            fname = os.path.join(self.folder,fname)               # folder and file name for the settings, to be tuned
        return fname                                              # return folder and file name for the servos settings





    def read_settings(self, fname=''):
        """Function reading the Cubotino_T_settings.txt file. Retrieves a dict with parameters and settings."""
        
        if fname == '':                                           # case fname is an empty string (default argument)
            fname = self.get_settings_fname()                     # folder and file name for the settings
        
        if os.path.exists(fname):                                 # case the settings file exists
            with open(fname, "r") as f:                           # settings file is opened in reading mode
                s = json.load(f)                                  # json file is parsed to a local dict variable
            
            # update key-values parameters, needed in case of additional parameters added at remote repo
            s = self.update_settings_file(fname, s)
            
            backup_fname = 'Cubotino_T_settings_backup.txt'       # filename for the backup
            self.save_backup(backup_fname, s)                     # settings are saved on a backup file
        else:                                                     # case the s file does not exists, or name differs
            print('Could not find the file: ', fname)             # feedback is printed to the terminal
            s = {}                                                # empty dict is assigned to s
        return s                                                  # settings dict is returned






    def parse_settings(self, s):
        """Function parsing the settings, for proper datatype check."""

        try:
            if s['frameless_cube'].lower().strip() == 'false':    # case frameless_cube parameter is a string == false
                frameless_cube = 'false'                          # cube with black frame around the facelets
            elif s['frameless_cube'].lower().strip() == 'true':   # case frameless_cube parameter is a string == true
                frameless_cube = 'true'                           # cube without black frame around the facelets
            elif s['frameless_cube'].lower().strip() == 'auto':   # case frameless_cube parameter is a string == auto
                frameless_cube = 'auto'                           # cube with/without black frame around the facelets
            else:                                                 # case the frameless parameter is not 'false', 'true' or 'auto'
                print('\n\nAttention: Wrong frameless_cube parameter: It should be "true", "false" or "auto".\n')  # feedback is printed to the terminal
                frameless_cube = 'auto'                           # cube with/without black frame around the facelets

            if s['disp_type'].lower().strip() == 'st7735':        # Slect Display type st7735 default
                s['disp_type'] = 'st7735'
            elif s['disp_type'].lower().strip() == 'st7789':      # Slect Display type st7789
                s['disp_type'] = 'st7789'
            else:
                print('\n\nAttention: Wrong disp_tyep parameter: It should be "st7735" or "st7789."\n')  # feedback is printed to the terminal
                s['disp_type'] = 'st7735'

            if s['disp_flip'].lower().strip() == 'false':          # Display not rotated.
                s['disp_flip'] = False
            elif s['disp_flip'].lower().strip() == 'true':         # Display flipped 180degrees
                s['disp_flip'] = True
            else:
                print('\n\nAttention: Wrong disp_flip parameter: It should be "true" or "false."\n')  # feedback is printed to the terminal
                s['disp_flip'] = False
            
            s['disp_width'] = int(s['disp_width'])                # display width, in pixels
            s['disp_height'] = int(s['disp_height'])              # display height, in pixels
            s['disp_offsetL'] = int(s['disp_offsetL'])            # Display offset on width, in pixels, Left if negative
            s['disp_offsetT'] = int(s['disp_offsetT'])            # Display offset on height, in pixels, Top if negative
            s['camera_width_res'] = int(s['camera_width_res'])    # Picamera resolution on width 
            s['camera_hight_res'] = int(s['camera_hight_res'])    # Picamera resolution on heigh (typo, but it's a var name since the start)
            s['s_mode'] = int(s['s_mode'])                        # camera setting mode (pixels binning)
            s['expo_shift'] = float(s['expo_shift'])              # camera exposure shift
            s['kl'] = float(s['kl'])                              # coefficient for PiCamera stability acceptance (l= lower)
            s['x_l'] = int(s['x_l'])                              # image crop on left (before warping)
            s['x_r'] = int(s['x_r'])                              # image crop on right (before warping)
            s['y_u'] = int(s['y_u'])                              # image crop on top (before warping)
            s['y_b'] = int(s['y_b'])                              # image crop on bottom (before warping)
            s['warp_fraction'] = float(s['warp_fraction'])        # coeff for warping the image
            
            s['warp_slicing'] = float(s['warp_slicing'])          # coeff for cropping the bottom warped image
            if s['warp_slicing'] == 0:                            # case the parameter equals to zero
                s['warp_slicing'] = 0.1                           # the parameter is set to 0.1
            
            s['square_ratio'] = float(s['square_ratio'])          # acceptance threshold for square sides difference
            s['rhombus_ratio'] = float(s['rhombus_ratio'])        # acceptance threshold for rhombus axes difference
            s['delta_area_limit'] = float(s['delta_area_limit'])  # acceptance threshold for facelet area dev from median
            s['sv_max_moves'] = int(s['sv_max_moves'])            # max moves requested to the Kociemba solver
            s['sv_max_time'] = float(s['sv_max_time'])            # timeout requested to the Kociemba solver
            s['collage_w'] = int(s['collage_w'])                  # image width for unfolded cube collage
            s['marg_coef'] = float(s['marg_coef'])                # cropping margin arounf faces for immage collage
            s['cam_led_bright'] = float(s['cam_led_bright'])      # PWM level for the 3W led at Top_cover
            s['detect_timeout'] = int(s['detect_timeout'])        # timeout for cube status detection
            s['show_time'] = int(s['show_time'])                  # showing time of the unfolded cube image collage
            s['warn_time'] = float(s['warn_time'])                # solve button pressing time before get a worning
            s['quit_time'] = float(s['quit_time'])                # solve button pressing time before the quit process
            s['vnc_delay'] = float(s['vnc_delay'])                # delay for cube moving to next face at scan (for VNC viewer)
            s['built_by'] = str(s['built_by'])                    # maker's name to add on the Cubotino logo
            s['built_by_x'] = int(s['built_by_x'])                # x coordinate for maker's name on display
            s['built_by_fs'] = int(s['built_by_fs'])              # font size for the maker's name on display
            s['fcs_delay'] = float(s['fcs_delay'])                # delay in secs to switch to Fix Coordinates System for facelets position
            
            if s['cover_self_close'].lower().strip() == 'false':  # case cover_self_close parameter is a string == false
                s['cover_self_close'] = False                     # cover_self_close parameter is set boolean False
            elif s['cover_self_close'].lower().strip() == 'true': # case cover_self_close parameter is a string == true
                s['cover_self_close'] = True                      # cover_self_close parameter is set boolean True
            else:                                                 # case the frameless parameter is not 'false', 'true' or 'auto'
                print('\n\nAttention: Wrong cover_self_close parameter: It should be "true" or "false."\n')  # feedback is printed to the terminal
                s['cover_self_close'] = False                     # cover_self_close parameter is set boolean False
            
            return s                                              # parsed settings dict is returned

        except:   # exception will be raised if json keys differs, or parameters cannot be converted (to float, int, string, etc)
            print("\n\n\n")                                       # print empty lines
            print("#"*83)                                         # print a separation line
            print(f"Error on converting imported parameters from {self.get_settings_fname()}") # feedback is printed to the terminal
            print("ChecK the file for non-numeric characters, in the settings supposed to have values.")  # feedback is printed to the terminal
            print("#"*83, "\n\n\n")                               # print a separation line
            sys.exit(1)                                           # script is quitted with error







    def read_servos_settings(self, fname=''):
        """Function reading the Cubotino_T_servo_settings.txt . Retrieves a dict with parameters and settings."""
        
        if fname == '':                                           # case fname is an empty string (default argument)
            fname = self.get_servo_settings_fname()               # fname for the text file to retrieve settings
        
        if os.path.exists(fname):                                 # case the servo_settings file exists
            with open(fname, "r") as f:                           # servo_settings file is opened in reading mode
                servo_s = json.load(f)                            # json file is parsed to a local dict variable
            
            # update key-values parameters, for additional parameters added at remote repo after first release
            servo_s = self.update_servos_settings_file(fname, servo_s, json)
            
            backup_fname = 'Cubotino_T_servo_settings_backup.txt' # filename for the backup
            self.save_backup(backup_fname, servo_s)               # servos settings are saved on a backup file      
        else:                                                     # case the servo_settings file does not exists, or name differs
            print('Could not find Cubotino_T_servo_settings.txt') # feedback is printed to the terminal                                  
            servo_s = {}                                          # empty dict is assigned to servo_s
        return servo_s                                            # return empty dict





    def parse_servos_settings(self, servo_s):
        """Function parsing the servos settings, for proper datatype check."""
        try:
            for key, value in servo_s.items():                    # iteration through servo_s dict
                servo_s[key] = float(value)                       # values are conferted to float
            return servo_s                                        # return servos settings dict
        
        except:   # exception will be raised if json keys differs, or parameters cannot be converted to float
            print("\n\n\n")                                       # print empty lines
            print("#"*78)                                         # print a separation line
            print(f"Error on converting imported parameters from {self.get_servo_settings_fname()}") # feedback is printed to the terminal
            print("Double checK the file for wrong characters in the settings (i.e., non-float).")   # feedback is printed to the terminal
            print("#"*78, "\n\n\n")                               # print a separation line
            sys.exit(1)                                           # script is quitted with error
                
        # below the previous approach (kept asz comment in case a non-float setting will be add in future)
        
#             servo_s['t_min_pulse_width'] = float(servo_s['t_min_pulse_width'])  # defines the min Pulse With the top servo reacts to
#             servo_s['t_max_pulse_width'] = float(servo_s['t_max_pulse_width'])  # defines the max Pulse With the top servo reacts to
#             servo_s['t_servo_close'] = float(servo_s['t_servo_close'])          # Top_cover close position, in gpiozero format
#             servo_s['t_servo_open'] = float(servo_s['t_servo_open'])            # Top_cover open position, in gpiozero format
#             servo_s['t_servo_read'] = float(servo_s['t_servo_read'])            # Top_cover camera read position, in gpiozero format
#             servo_s['t_servo_flip'] = float(servo_s['t_servo_flip'])            # Top_cover flip position, in gpiozero format
#             servo_s['t_servo_rel_delta'] = float(servo_s['t_servo_rel_delta'])  # Top_cover release angle movement from the close position to release tension
#             servo_s['t_flip_to_close_time'] = float(servo_s['t_flip_to_close_time'])  # time for Top_cover from flip to close position
#             servo_s['t_close_to_flip_time'] = float(servo_s['t_close_to_flip_time'])  # time for Top_cover from close to flip position 
#             servo_s['t_flip_open_time'] = float(servo_s['t_flip_open_time'])    # time for Top_cover from open to flip position, and viceversa  
#             servo_s['t_open_close_time'] = float(servo_s['t_open_close_time'])  # time for Top_cover from open to close position, and viceversa
#             servo_s['t_rel_time'] = float(servo_s['t_rel_time'])                # time for Top_cover to release tension from close position
# 
#             servo_s['b_min_pulse_width'] = float(servo_s['b_min_pulse_width'])  # defines the min Pulse With the bottom servo reacts to
#             servo_s['b_max_pulse_width'] = float(servo_s['b_max_pulse_width'])  # defines the max Pulse With the bottom servo reacts to
#             servo_s['b_servo_CCW'] = float(servo_s['b_servo_CCW'])              # Cube_holder max CCW angle position
#             servo_s['b_servo_CW'] = float(servo_s['b_servo_CW'])                # Cube_holder max CW angle position
#             servo_s['b_home'] = float(servo_s['b_home'])                        # Cube_holder home angle position
#             servo_s['b_rel_CCW'] = float(servo_s['b_rel_CCW'])                  # Cube_holder release angle from CCW angle positions, to release tension
#             servo_s['b_rel_CW'] = float(servo_s['b_rel_CW'])                    # Cube_holder release angle from CW angle positions, to release tension
#             servo_s['b_extra_home_CW'] = float(servo_s['b_extra_home_CW'])      # Cube_holder release angle from home angle positions, to release tension
#             servo_s['b_extra_home_CCW'] = float(servo_s['b_extra_home_CCW'])    # Cube_holder release angle from home angle positions, to release tension
#             servo_s['b_spin_time'] = float(servo_s['b_spin_time'])              # time for Cube_holder to spin 90 deg (cune not contrained)
#             servo_s['b_rotate_time'] = float(servo_s['b_rotate_time'])          # time for Cube_holder to rotate 90 deg (cube constrained)
#             servo_s['b_rel_time'] = float(servo_s['b_rel_time'])                # time for Cube_holder to release tension at home, CCW and CW positions
#             return servo_s                                                      # return servos settings dict
                

        
        
        






    def save_setting(self, fname, data, debug=False):
        """Saves the settings s."""
        if debug:                                                # case debug variable is set true
            print('\nImporting servos settings from the text file:', fname)  # feedback is printed to the terminal
            print('\nImported parameters: ')                     # feedback is printed to the terminal
            for parameter, setting in data.items():              # iteration over the settings dict
                print(parameter,': ', setting)                   # feedback is printed to the terminal
            print()                                              # prints an empty line
            
        for key, value in data.items():                          # iteration through the dict data
            data[key]=str(value)                                 # values of data are converted to string
        
        os.umask(0) # The default umask is 0o22 which turns off write permission of group and others
        backup_fname = os.path.join(self.folder, fname)          # folder and file name for the settings backup

        with open(os.open(backup_fname, os.O_CREAT | os.O_WRONLY, 0o777), 'w') as f:  # settings_backup file is opened in writing mode
            if debug:                                            # case debug variable is set true
                print('Copy of servos settings parameter is saved as backup at: ', backup_fname)  # feedback is printed to the terminal
            
            f.write(json.dumps(data, indent=0))                  # content of the setting file is saved in another file, as backup
            f.truncate()                       # truncates the file (prevents older characters at file-end, if new content is shorter)
        
        if "servo" in fname:                                     # case fname contains 'servo'
            self.servo_s = self.parse_servos_settings(data)      # servos settings datatypes are parsed
        else:                                                    # case fname does not contain 'servo'
            self.s = self.parse_settings(data)                   # settings datatypes are parsed






    def save_backup(self, fname, data):
        """Saves a backup copy of the settings data."""
        
        for key, value in data.items():
            data[key]=str(value)
        os.umask(0) # The default umask is 0o22 which turns off write permission of group and others
        backup_fname = os.path.join(self.folder, fname)          # folder and file name for the settings backup
        with open(os.open(backup_fname, os.O_CREAT | os.O_WRONLY, 0o777), 'w') as f:  # settings_backup file is opened in writing mode
            f.write(json.dumps(data, indent=0))                  # content of the setting file is saved in another file, as backup






    def load_previous_settings(self, servo=False):
        """ Function loading the settings from latest backup file saved.
            The argument servo allows to load the servo setting or the (cam) settings."""
        
        if servo:                                                # case the boolean servo is set True
            fname = self.get_servo_settings_fname()              # fname for the servo settings file to retrieve settings
        else:                                                    # case the boolean servo is set False
            fname = self.get_settings_fname()                    # fname for the settings file to retrieve settings
        
        backup_fname = fname[:-4] + '_backup*.txt'               # backup filename is made
        backup_files = sorted(glob.iglob(backup_fname), key=os.path.getmtime, reverse=True) # ordered list of backuped settings files 
        
        if len(backup_files) > 0:                                # case there are backup setting files
            if len(backup_files) > 10:                           # case there are more than 10 backup files
                while len(backup_files) > 10:                    # iteration until there will only be 10 backup files
                    os.remove(backup_files[-1])                  # the oldes backup file is deleted
                    backup_files = sorted(glob.iglob(backup_fname), key=os.path.getctime, reverse=True) # ordered list of backuped settings files     
            latest_backup = backup_files[0]                      # latest settings files backed up
            latest_backup = os.path.join(self.folder, latest_backup)  # folder and file name for the settings backup to upload
            if servo:                                            # case the boolean servo is set True
                servo_s = self.read_servos_settings(fname=latest_backup) # servos settings are read from the latest_backup file
                self.servo_s = self.parse_servos_settings(servo_s) # servos settings datatypes are parsed
                print(f"\nUploaded servos settings from latest_backup:  {latest_backup}")  # feedback is printed to the terminal
                return self.servo_s                              # settings dict, from the Json file, is returned
            else:                                                # case the boolean servos is set False
                s = self.read_settings(fname=latest_backup)      # settings are read from the latest_backup file
                self.s = self.parse_settings(s)                  # settings datatypes are parsed
                print(f"\nUploaded cam settings from latest_backup:  {latest_backup}")  # feedback is printed to the terminal
                return self.s                                    # settings dict, from previous backup, is returned
        
        else:                                                    # case there aren't backup setting files
            if servo:                                            # case the boolean servo is set True
                servo_s = self.read_servos_settings()            # servos settings are read from the fname file
                self.servo_s = self.parse_servos_settings(servo_s)  # servos settings datatypes are parsed
                return self.servo_s                              # settings dict, from the Json file, is returned
            else:                                                # case the boolean servos is set False
                s = self.read_settings()                         # settings are read from the fname file
                self.s = self.parse_settings(s)                  # settings datatypes are parsed
                print(f"\nNot found backup files, uploaded settings from file: {fname}")
                return self.s                                    # settings dict, from the Json file, is returned
            






    def backups_cleanup(self, fname, n):
        """Function that keep the last n backup files."""
        
        backup_fname = fname[:-4] + '_backup*.txt'               # common name prefix for the backup files
        backup_files = sorted(glob.iglob(backup_fname), key=os.path.getmtime, reverse=True) # ordered list of backuped settings files 
        if len(backup_files) > n:                                # case there are more than n backup files
            while len(backup_files) > n:                         # iteration until there will only be n backup files
                os.remove(backup_files[-1])                      # the oldest backup file is deleted
                print(f"Erased the {n+1} oldest backup file:{backup_files[-1]}")  # feedback is printed to the terminal
                backup_files = sorted(glob.iglob(backup_fname), key=os.path.getctime, reverse=True) # ordered list of backuped settings files 






    def update_settings_file(self, fname, s):
        """Function to check if the existing fname (Cubotino_T_settings.txt) has all
        the parameters that were instroduced after the first release.
        Cubotino_T_settings.txt file will be updated with eventually missed parameters
        and related default values.
        """

        s_keys = s.keys()
        any_change = False
        
        if 'frameless_cube' not in s_keys:
            s['frameless_cube']='false'
            any_change = True
            
        if 'cover_self_close' not in s_keys:
            s['cover_self_close']='false'
            any_change = True
            
        if 's_mode' not in s_keys:
            s['s_mode']='7'
            any_change = True
            
        if 'vnc_delay' not in s_keys:
            s['vnc_delay']= '0.5'
            any_change = True
        
        if 'built_by' not in s_keys:
            s['built_by']=''
            any_change = True
            
        if 'built_by_x' not in s_keys:
            s['built_by_x']='25'
            any_change = True
            
        if 'built_by_fs' not in s_keys:
            s['built_by_fs']='16'
            any_change = True
        
        if 'expo_shift' not in s_keys:
            s['expo_shift']='-0.5'        # this parameter is ignored on OS10 (Buster)
            any_change = True
        
        if 'fcs_delay' not in s_keys:
            s['fcs_delay']='3'
            any_change = True
        
        if 'disp_type' not in s_keys:
            s['disp_type'] = 'st7735'
            any_change = True
        
        if 'disp_flip' not in s_keys:
            s['disp_flip'] = 'false'
            any_change = True
         
        if any_change:
            print('\nOne time action: Adding new parameters to the Cubotino_T_settings.txt')
            print('Action necessary for compatibility with the latest downloaded Cubotino_T.py \n')
            with open(fname, 'w') as f:
                f.write(json.dumps(s, indent=0))   # content of the updated setting is saved
        
        return s






    def update_servos_settings_file(self, fname, servo_s, json):
        """Function to check if the existing setting file(fname: Cubotino_T_servos_settings.txt)
            has all the parameters that were introduced after first release.
            Cubotino_T_servos_settings.txt will be eventually updated with missed parameters
            and related default values.
            """

        s_keys = servo_s.keys()
        any_change = False
        
        if 'b_extra_home' in s_keys:
            if 'b_extra_home_CCW' not in s_keys:
                # Cube_holder release angle at home positions, to release tension, when rotating from CCW
                servo_s['b_extra_home_CCW'] = servo_s['b_extra_home'] # extra home rotation when rotating from CCW
                any_change = True
            if 'b_extra_home_CW' not in s_keys:
                # Cube_holder release angle at home positions, to release tension, when rotating from CW
                servo_s['b_extra_home_CW'] = servo_s['b_extra_home']  # extra home rotation when rotating from CW
                any_change = True
            del servo_s['b_extra_home']   # setting used up to 05 April 2023 is deleted
        
        if 'b_extra_sides' in s_keys:
            if 'b_rel_CCW' not in s_keys:
                servo_s['b_rel_CCW'] = servo_s['b_extra_sides'] # backward rotation from CCW
                any_change = True
            if 'b_rel_CW' not in s_keys:
                servo_s['b_rel_CW'] = servo_s['b_extra_sides']  # backward rotation from CW
                any_change = True
            del servo_s['b_extra_sides']   # setting used up to 05 April 2023 is deleted
            
        if 't_rel_time' not in s_keys: 
            servo_s['t_rel_time'] = float(0)  # time for Top_cover to release tension from close position
            any_change = True
        
        if any_change:
            print('\nOne time action: Adding new parameters to the Cubotino_T_settings.txt')
            print('Action necessary for compatibility with the latest downloaded Cubotino_T.py \n')
            with open(fname, 'w') as f:
                f.write(json.dumps(servo_s, indent=0))   # content of the updated setting is saved
        
        return servo_s





settings = Settings()

if __name__ == "__main__":
    """the main function is just used to check if the settings loading works properly. """
    
    print()
    print(f"Settings from {settings.get_settings_fname()}:")
    s = settings.get_settings()
    for key, value in s.items():
        print(key, ":", value)
    print()
    
    print()
    print(f"Settings from {settings.get_servo_settings_fname()}:")
    s = settings.get_servos_settings()
    for key, value in s.items():
        print(key, ":", value)
    print()

