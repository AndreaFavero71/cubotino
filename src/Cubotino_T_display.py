#!/usr/bin/python
# coding: utf-8

from PIL import Image, ImageDraw, ImageFont  # classes from PIL for image manipulation
import ST7735                                # library for the TFT display with ST7735 driver 
import os.path, pathlib, json                # library for the json parameter parsing for the display
from getmac import get_mac_address           # library to get the device MAC ddress

class Display:
    def __init__(self):
        """ Imports and set the display (128 x 160 pixels) https://www.az-delivery.de/it/products/1-77-zoll-spi-tft-display.
            In my case was necessary to play with offset and display dimensions (pixels) to get an acceptable result.
            For a faster display init, check the Tuning chapter of the project instructions."""
                
        # convenient choice for Andrea Favero, to upload the settings fitting my robot, via mac check
        folder = pathlib.Path().resolve()                             # active folder (should be home/pi/cube)  
        eth_mac = get_mac_address()                                   # mac address is retrieved
        if eth_mac == 'e4:5f:01:8d:59:97':                            # case the script is running on AF (Andrea Favero) robot
            fname = os.path.join(folder,'Cubotino_T_settings_AF.txt') # AF robot settings (do not use these at the start)
        else:                                                         # case the script is not running on AF (Andrea Favero) robot
            fname = os.path.join(folder,'Cubotino_T_settings.txt')    # folder and file name for the settings, to be tuned
        
        if os.path.exists(fname):                                     # case the settings file exists
            with open(fname, "r") as f:                               # settings file is opened in reading mode
                settings = json.load(f)                               # json file is parsed to a local dict variable
            try:
                disp_width = int(settings['disp_width'])              # display width, in pixels
                disp_height = int(settings['disp_height'])            # display height, in pixels
                disp_offsetL = int(settings['disp_offsetL'])          # Display offset on width, in pixels, Left if negative
                disp_offsetT = int(settings['disp_offsetT'])          # Display offset on height, in pixels, Top if negative
            except:
                print('error on converting imported parameters to int') # feedback is printed to the terminal
        else:                                                           # case the settings file does not exists, or name differs
            print('could not find the file: ', fname)                   # feedback is printed to the terminal 
        
        #(AF )  # NOTE: On my display text is parallel to the display if width set to 125 pxl instead of 128
        #(AF )  # To solve this issue, further than death pixels on two display sides, different width/height and offsets are used 
        self.disp = ST7735.ST7735(port=0, cs=0,                              # SPI and Chip Selection                  
                            dc=27, rst=22, backlight=4,                # GPIO pins used for the SPI, reset and backlight control
                            width=disp_width,            #(AF 132)     # see note above for width and height !!!
                            height=disp_height,          #(AF 162)     # see note above for width and height !!!                         
                            offset_left=disp_offsetL,                  # see note above for offset  !!!
                            offset_top= disp_offsetT,                  # see note above for offset  !!!
                            rotation=270,                              # image orientation
                            invert=False, spi_speed_hz=10000000)       # image invertion, and SPI
        
        self.disp.set_backlight(0)                                           # display backlight is set off
        self.disp_w = self.disp.width                                             # display width, retrieved by display setting
        self.disp_h = self.disp.height                                            # display height, retrieved by display setting
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h),color=(0, 0, 0))   # display image generation, full black
        self.disp.display(disp_img)                                          # image is displayed        

    def set_backlight(self, value):
        """Set the backlight on/off."""
        self.disp.set_backlight(value)


    def clean_display(self):
        """ Cleans the display by settings all pixels to black."""

        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=(0, 0, 0))  # full black screen as new image
        self.disp.display(disp_img)                                          # display is shown to display


    def show_on_display(self, r1,r2,x1=20,y1=25,x2=20,y2=65,fs1=22,fs2=22):
        """Shows text on two rows, with parameters to generalize this function"""
        
        font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", fs1)  # font and size for first text row
        font2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", fs2)  # font and size for second text row
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=(0, 0, 0)) 
        disp_draw = ImageDraw.Draw(disp_img)
        disp_draw.text((x1, y1), r1, font=font1, fill=(255, 255, 255))    # first text row start coordinate, text, font, white color
        disp_draw.text((x2, y2), r2, font=font2, fill=(255, 255, 255))    # second text row start coordinate, text, font, white color
        self.disp.display(disp_img)                                       # image is plot to the display

    def display_progress_bar(self, percent):
        """ Function to print a progress bar on the display."""

        # percent value printed as text 
        fs = 40                 # font size
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", fs)     # font and its size
        text_x = int(self.disp_w/2 - (fs*len(str(percent))+1)/2)                   # x coordinate for the text starting location         
        text_y = 15                                                           # y coordinate for the text starting location
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=(0, 0, 0)) 
        disp_draw = ImageDraw.Draw(disp_img)
        disp_draw.text((text_x, text_y), str(percent)+'%', font=font, fill=(255, 255, 255))    # text with percent value
        
        # percent value printed as progress bar filling 
        x = 15                  # x coordinate for the bar starting location
        y = 60                  # y coordinate for the bar starting location
        gap = 3                 # gap in pixels between the outer border and inner filling (even value is preferable) 
        barWidth = 35           # width of the bar, in pixels
        barLength = 135         # lenght of the bar, in pixels
        filledPixels = int( x+gap +(barLength-2*gap)*percent/100)  # bar filling length, as function of the percent
        disp_draw.rectangle((x, y, x+barLength, y+barWidth), outline="white", fill=(0,0,0))     # outer bar border
        disp_draw.rectangle((x+gap, y+gap, filledPixels-1 , y+barWidth-gap), fill=(255,255,255)) # bar filling
        
        self.disp.display(disp_img) # image is plotted to the display


    def show_cubotino(self):
        """ Shows the Cubotino logo on the display."""
                
        image = Image.open("Cubotino_T_Logo_265x212_BW.jpg")  # opens the CUBOTino logo image (jpg file, converted and saved from pdf file)
        image = image.resize((self.disp_w, self.disp_h))                # resizes the image to match the display.
        self.disp.display(image)                                   # draws the image on the display hardware.
        
display = Display()

if __name__ == "__main__":
    display.set_backlight(0)  
    display.show_cubotino() 
    display.set_backlight(1)  