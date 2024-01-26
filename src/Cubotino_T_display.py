#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
#  Andrea Favero 22 January 2024
#
# This script relates to CUBOTino autonomous, a small and simple Rubik's cube solver robot 3D printed
# This specific script manages the display, and it's imported by Cubotino_T.py and Cubotino_T_servos.py
#
#############################################################################################################
"""


from PIL import Image, ImageDraw, ImageFont  # classes from PIL for image manipulation
import ST7735                                # library for the TFT display with ST7735 driver
import os.path, pathlib, json                # library for the json parameter parsing for the display
from getmac import get_mac_address           # library to get the device MAC ddress
from get_macs_AF import get_macs_AF          # import the get_macs_AF function
from Cubotino_T_pigpiod import pigpiod as pigpiod # start the pigpiod server


class Display:
    display_initialized = False
    def __init__(self):
        """ Imports and set the display (128 x 160 pixels) https://www.az-delivery.de/it/products/1-77-zoll-spi-tft-display.
            In my (AF) case was necessary to play with offset and display dimensions (pixels) to get an acceptable result.
            For a faster display init: Reduce the time.sleep to 0.1 (3 times, iso 0.5) at reset at __init__ of ST7735 module."""
            
        
        if not self.display_initialized:
            # convenient choice for Andrea Favero, to upload the settings fitting my robot, via mac check
            macs_AF = get_macs_AF()                                   # mac addresses of AF bots are retrieved
            folder = pathlib.Path().resolve()                         # active folder (should be home/pi/cube)  
            eth_mac = get_mac_address().lower()                       # mac address is retrieved
            fname = 'Cubotino_T_settings.txt'                         # file name with settings for the display
            if eth_mac in macs_AF:                                    # case the script is running on AF (Andrea Favero) robot
                pos = macs_AF.index(eth_mac)                          # mac address row position in macs_AF.txt file
                fname = self.get_fname_AF(fname, pos)                 # AF robot settings file name (do not use these at the start)
            else:                                                     # case the script is not running on AF (Andrea Favero) robot
                fname = os.path.join(folder,fname)                    # folder and file name for the settings, to be tuned
            self.display_settings = False
            if os.path.exists(fname):                                 # case the settings file exists
                with open(fname, "r") as f:                           # settings file is opened in reading mode
                    settings = json.load(f)                           # json file is parsed to a local dict variable
                try:
                    self.disp_width = int(settings['disp_width'])     # display width, in pixels
                    self.disp_height = int(settings['disp_height'])   # display height, in pixels
                    self.disp_offsetL = int(settings['disp_offsetL']) # Display offset on width, in pixels, Left if negative
                    self.disp_offsetT = int(settings['disp_offsetT']) # Display offset on height, in pixels, Top if negative
                    self.built_by = str(settings['built_by'])         # maker's name to add on the Cubotino logo
                    self.built_by_x = int(settings['built_by_x'])     # x coordinate for maker's name on display
                    self.built_by_fs = int(settings['built_by_fs'])   # font size for the maker's name on display
                    self.display_settings = True                      # display_settings is set True
                except:
                    print('Error on converting imported parameters to int') # feedback is printed to the terminal
            else:                                                     # case the settings file does not exists, or name differs
                print('Could not find the file: ', fname)             # feedback is printed to the terminal 
        
        if self.display_settings:
            self.disp = ST7735.ST7735(port=0, cs=0,                   # SPI and Chip Selection                  
                                dc=27, rst=22, backlight=4,           # GPIO pins used for the SPI, reset and backlight control
                                width=self.disp_width,     #(AF 132)  # see note above for width and height !!!
                                height=self.disp_height,   #(AF 162)  # see note above for width and height !!!                         
                                offset_left=self.disp_offsetL,        # see note above for offset  !!!
                                offset_top=self.disp_offsetT,         # see note above for offset  !!!
                                rotation=270,                         # image orientation
                                invert=False,                         # image invertion,
                                spi_speed_hz=10000000)                # SPI frequence
        
            self.disp.set_backlight(0)                                # display backlight is set off
            self.disp_w = self.disp.width                             # display width, retrieved by display setting
            self.disp_h = self.disp.height                            # display height, retrieved by display setting
            disp_img = Image.new('RGB', (self.disp_w, self.disp_h),color=(0, 0, 0))   # display image generation, full black
            self.disp.display(disp_img)                               # image is displayed
            if not self.display_initialized:                          # case display_initialized is set False
                print("\nDisplay initialized\n")                      # feedback is printed to the terminal
                self.display_initialized = True                       # display_initialized is set True




    def set_backlight(self, value):
        """Set the backlight on/off."""
        self.disp.set_backlight(value)




    def clean_display(self):
        """ Cleans the display by settings all pixels to black."""
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=(0, 0, 0))  # full black screen as new image
        self.disp.display(disp_img)                                               # display is shown to display




    def show_on_display(self, r1,r2,x1=20,y1=25,x2=20,y2=65,fs1=22,fs2=22):
        """Shows text on two rows, with parameters to generalize this function; Parameters are
            r1, r2: text for row1 and row2
            x1, x2: x coordinate for text at row1 and row2
            y1, y2: y coordinate for text at row1 and row2
            fs1, fs2: font size for text at row1 and row2
            """
        
        font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", fs1)  # font and size for first text row
        font2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", fs2)  # font and size for second text row
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=(0, 0, 0))                 # full black image
        disp_draw = ImageDraw.Draw(disp_img)                                                     # image is drawned
        disp_draw.text((x1, y1), r1, font=font1, fill=(255, 255, 255))    # first text row start coordinate, text, font, white color
        disp_draw.text((x2, y2), r2, font=font2, fill=(255, 255, 255))    # second text row start coordinate, text, font, white color
        self.disp.display(disp_img)                                       # image is plot to the display




    def display_progress_bar(self, percent, scrambling=False):
        """ Function to print a progress bar on the display."""
        
        w = self.disp_w                                            # display width, retrieved by display setting
        
        # percent value printed as text 
        fs = 40                 # font size
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", fs)     # font and its size
        text_x = int(w/2 - (fs*len(str(percent))+1)/2)             # x coordinate for the text starting location         
        text_y = 15                                                # y coordinate for the text starting location
        disp_img = Image.new('RGB', (w, self.disp_h), color=(0, 0, 0)) # full black image
        disp_draw = ImageDraw.Draw(disp_img)                        # image is drawned
        disp_draw.text((text_x, text_y), str(percent)+'%', font=font, fill=(255, 255, 255))    # text with percent value
        
        # percent value printed as progress bar filling 
        x = 15                  # x coordinate for the bar starting location
        y = 60                  # y coordinate for the bar starting location
        gap = 3                 # gap in pixels between the outer border and inner filling (even value is preferable) 
        if not scrambling:      # case the robot is solving a cube
            barWidth = 35       # width of the bar, in pixels
        elif scrambling:        # case the robot is scrambling a cube
            barWidth = 18       # width of the bar, in pixels
            fs = 18             # font size
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", fs)  # font and its size
            disp_draw.text((15, 85), 'SCRAMBLING', font=font, fill=(255, 255, 255))                # SCRAMBLING text
            
        barLength = w-2*x-4 #135   # lenght of the bar, in pixels
        filledPixels = int( x+gap +(barLength-2*gap)*percent/100)  # bar filling length, as function of the percent
        disp_draw.rectangle((x, y, x+barLength, y+barWidth), outline="white", fill=(0,0,0))      # outer bar border
        disp_draw.rectangle((x+gap, y+gap, filledPixels-1 , y+barWidth-gap), fill=(255,255,255)) # bar filling
        
        self.disp.display(disp_img) # image is plotted to the display




    def show_cubotino(self, built_by='', x=25, fs=22):
        """ Shows the Cubotino logo on the display."""
                
        image = Image.open("Cubotino_T_Logo_265x212_BW.jpg")       # opens the CUBOTino logo image (jpg file)
        image = image.resize((self.disp_w, self.disp_h))           # resizes the image to match the display.
        
        if built_by != '':                                         # case the built_by variable is not empty
            disp_draw = ImageDraw.Draw(image)                      # image is plotted to display
            font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)  # font1
            disp_draw.text((15, 10), "Andrea FAVERO's", font=font1, fill=(0, 0, 255))  # first row text test
            
            font2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 10)  # font1
            disp_draw.text((60, 85), "Built by", font=font2, fill=(255, 255, 255))    # second row text test
            
            font3 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", fs)  # font1
            disp_draw.text((x, 104), built_by, font=font3, fill=(255, 0, 0))              # third row text test
        self.disp.display(image)                                   # draws the image on the display hardware.




    def show_face(self, side, colours=[]):
        """ Function to print a sketch of the cube face colours."""
        
        w = self.disp_w                                    # display width, retrieved by display setting
        h = self.disp_h                                    # display height, retrieved by display setting
        faces = ('', 'U', 'B', 'D', 'F', 'R', 'L')         # tuple of faces letters
        y_start = 20                                       # y coordinate for face top-left corner
        d = int(h-y_start)/3.8                             # facelet square side
        x_start = w-5-3*d                                  # x coordinate for face top-left corner
        gap = 3                                            # gap beftween the facelets border and facelets coloured part
        
        font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 17)  # font1
        font2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 70)  # font1
        disp_img = Image.new('RGB', (w, h), color=(0, 0, 0))  # full black image
        disp_draw = ImageDraw.Draw(disp_img)               # image is drawned
        disp_draw.text((12, 25), 'FACE', font=font1, fill=(255, 255, 255))   # first row text test
        disp_draw.text((9, 40), faces[side], font=font2, fill=(255, 255, 255))   # first row text test
        self.disp.set_backlight(1)                         # display backlight is set on
        
        fclt = 0
        y = y_start                                        # y coordinate value for the first 3 facelets
        for i in range(3):                                 # iteration over rows
            x = x_start                                    # x coordinate value for the first facelet
            for j in range(3):                             # iteration over columns
                disp_draw.rectangle((x, y, x+d, y+d), outline="white", fill=(0,0,0))   # outer bar border
                if len(colours)==9:                        # case colour is prodided
                    disp_draw.rectangle((x+gap, y+gap, x+d-gap-1, y+d-gap-1), colours[fclt]) #colours[fclt]) # bar filling
                x = x+d                                    # x coordinate is increased by square side
                if j == 2: y = y+d                         # once at the third column the row is incremented
                fclt+=1
        
        self.disp.display(disp_img) # image is plotted to the display

    
    
    def test1_display(self):
        """ Test showing some text into some rectangles."""
        
        print("Display test for 20 seconds")
        print("Display shows rectangles, text and Cubotino logo")
        
        import time
        
        w = self.disp_w                                            # display width, retrieved by display setting
        h = self.disp_h                                            # display height, retrieved by display setting
        
        font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)  # font1
        font2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)  # font2
        disp_img = Image.new('RGB', (w, h), color=(0, 0, 0))      # full black image
        
        self.disp.set_backlight(1)                                        # display backlight is set on
        start = time.time()                                               # time refrence for countdown
        timeout = 20.5                                                    # timeout
        while time.time() < start + timeout:                              # while loop until timeout
            
            t_left = int(round(timeout + start - time.time(),0))          # time left
            
            if t_left%2==0:                                               # case the time left is even
                t_left_str = str(t_left)                                  # string of time left
                pos = w-40 if t_left>9 else w-35                          # position x for timeout text
                disp_img = Image.new('RGB', (w, h), color=(0, 0, 0))      # full black image
                disp_draw = ImageDraw.Draw(disp_img)                      # image is drawned

                disp_draw.rectangle((2, 2, w-4, h-4), outline="white", fill=(0,0,0))    # border 1
                disp_draw.rectangle((5, 5, w-7, h-7), outline="white", fill=(0,0,0))    # border 2
                disp_draw.rectangle((8, 8, w-10, h-10), outline="white", fill=(0,0,0))  # border 3
                disp_draw.rectangle((w-45, h-40, w-14, h-14), outline="blue", fill=(0,0,0)) # border for timeout
                
                disp_draw.text((pos, h-36), t_left_str , font=font2, fill=(0, 0, 255))  # timeout text
                disp_draw.text((30, 25), 'DISPLAY', font=font1, fill=(255, 255, 255))   # first row text test
                disp_draw.text((33, 75), 'TEST', font=font1, fill=(255, 255, 255))      # second row text test
                self.disp.display(disp_img)                                             # image is drawned
                time.sleep(0.1)                                           # little sleeping time   
            else:                                                         # case the time left is odd
                self.show_cubotino(self.built_by, self.built_by_x, self.built_by_fs) # cubotino logo is displayed
                time.sleep(0.1)                                           # little sleeping time
        
        if time.time() >= start + timeout:                                # case the while loop hasn't been interrupted
            time.sleep(1)                                                 # little sleeping time
        
        self.show_cubotino(self.built_by, self.built_by_x, self.built_by_fs)  # cubotino logo is show to display
        time.sleep(2)                                                     # little delay
        self.clean_display()                                              # display is set to full black
        self.disp.set_backlight(0)                                        # display backlight is set off
        print("Display test1 finished\n")                                 # feedback is printed to the terminal




    def test2_display(self):
        """ Test showing the Western Rubik's cube face colours."""
        
        print("\nDisplay URF colours sequence of Western Rubik's")
        import time
        colors_bgr = {'white':(255,255,255), 'red':(0,0,204), 'green':(0,132,0),
                      'yellow':(0,245,245), 'orange':(0,128,255), 'blue':(204,0,0)}   # bright colors assigned to the six faces colors
        colors=('white', 'red', 'green', 'yellow', 'orange', 'blue')  # URF colours sequence in Western Rubik's cube
        for f in range(6):                                            # iteration over 6 (six cube's faces)
            print(colors[f])                                          # feedback is printed to the terminal
            bgr=[]                                                    # empty list to hold the bgr of the 9 facelets
            for i in range(9):                                        # iteration over 9 (none cube's face facelets)
                bgr.append(colors_bgr[colors[f]])                     # list is populated with the related face's colours
            display.show_face(f+1, bgr)                               # display shows the face letter and facelets colours
            time.sleep(3)                                             # sleep time to let user time to evaluate the display
        self.clean_display()                                          # display is set to full black
        self.disp.set_backlight(0)                                    # display backlight is set off
        print("Display test2 finished\n")                             # feedback is printed to the terminal




    def get_fname_AF(self, fname, pos):
        return fname[:-4] + '_AF' + str(pos+1) + '.txt'






display = Display()

if __name__ == "__main__":
    """the main function can be used to test the display. """
    
    display.test1_display()
    display.test2_display()
    display.set_backlight(0)


##### test show_face, random colors #####
#     import random
#     import time
#     colors_bgr = {'white':(255,255,255), 'red':(0,0,204), 'green':(0,132,0),
#              'yellow':(0,245,245), 'orange':(0,128,255), 'blue':(204,0,0)}   # bright colors assigned to the six faces colors
#     colors=('white', 'red', 'green', 'yellow', 'orange', 'blue')
#     for f in range(6):
#         bgr=[]
#         for i in range(9):
#             ref=(colors_bgr[colors[random.randint(0,5)]])
#             bgr.append(ref)
#         display.show_face(f+1,bgr)
#         time.sleep(1)
#     display.set_backlight(0)
#########################################

    
