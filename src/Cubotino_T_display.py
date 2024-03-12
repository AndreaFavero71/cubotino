#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
#  Andrea Favero 10 march 2024
#
# This script relates to CUBOTino autonomous, a small and simple Rubik's cube solver robot 3D printed
# This specific script manages the display, and it's imported by other Cubotino_T scripts.
#
#############################################################################################################
"""



from Cubotino_T_settings_manager import settings as settings   # custom library managing the settings from<>to the settings files
from Cubotino_T_pigpiod import pigpiod as pigpiod  # start the pigpiod server
from PIL import Image, ImageDraw, ImageFont        # classes from PIL for image manipulation
import os.path, pathlib                            # libraries for path management


# Dictionary of LCD data, ST7735 display parameters are default for all other conversions.
#    Colors: <BGR|RGB> What color scheme does the display use
#    Inverted: <True|False> Are white and black inverted
#    BaseRotation: <180|270> Sets the default rotation
#    SPI: <Value 3900000-125000000> Spi bus speed.
#    FontScale: Scale factor to apply to all fonts.
LCDdata = { 'st7735': {'Colors': 'BGR', 'Inverted': False, 'BaseRotation': 270, 'SPI': 10000000, 'FontScale': 1.0, },
            'st7789': {'Colors': 'RGB', 'Inverted': True,  'BaseRotation': 180, 'SPI': 50000000, 'FontScale': 2.0, }
          }
default_x = 160                                                           # Default X Resolution for scaling
default_y = 128                                                           # Default Y Resolution for scaling

class Display:
    display_initialized = False
    def __init__(self):
        """ Imports and set the display (128 x 160 pixels) https://www.az-delivery.de/it/products/1-77-zoll-spi-tft-display.
            In my (AF) case was necessary to play with offset and display dimensions (pixels) to get an acceptable result.
            For a faster display init: Reduce the time.sleep to 0.1 (3 times, iso 0.5) at reset at __init__ of ST7735 module."""
            
        if not self.display_initialized:
            s = settings.get_settings()                               # settings are retrieved from the settings Class
            self.disp_type = s['disp_type']                           # Slect Display type st7735 default
            self.disp_width = int(s['disp_width'])                    # display width, in pixels
            self.disp_height = int(s['disp_height'])                  # display height, in pixels
            self.disp_offsetL = int(s['disp_offsetL'])                # Display offset on width, in pixels, Left if negative
            self.disp_offsetT = int(s['disp_offsetT'])                # Display offset on height, in pixels, Top if negative
            self.disp_flip = s['disp_flip']                           # Display flipped 180degrees
            self.built_by = str(s['built_by'])                        # maker's name to add on the Cubotino logo
            self.built_by_x = int(s['built_by_x'])                    # x coordinate for maker's name on display
            self.built_by_fs = int(s['built_by_fs'])                  # font size for the maker's name on display
            self.display_settings = True                              # display_settings is set True

        if not self.display_settings:                                 # case display_settings is still False
            print("Error on loading the display parameters at Cubotino_T_display")
            
        if self.disp_type == 'st7735':                                # Using a ST7735 based display.
            print('Importing ST7735 Display')
            from ST7735 import ST7735 as LCD                          # library for the TFT display with ST7735 driver
        elif self.disp_type == 'st7789':                              # Using a ST7789 based display. 
            print('Importing ST7789 Display')
            from ST7789 import ST7789 as LCD                          # library for the TFT display with ST7789 driver
        else:
            raise Exception('Invalid Display Type in settings')       # raise exception if display type is wrong

        # Set screen data from table above
        if self.disp_flip:                                            # Flip the screen upside down if true.
            self.disp_rotation = LCDdata[self.disp_type]['BaseRotation'] - 180  # LCD Data uses >= 180 for base.
        else:
            self.disp_rotation = LCDdata[self.disp_type]['BaseRotation']
            
        self.disp_color_invert = LCDdata[self.disp_type]['Inverted']  # Invert white/black
        self.disp_spi_freq = LCDdata[self.disp_type]['SPI']           # Set SPI Speed
        self.disp_colors = LCDdata[self.disp_type]['Colors']          # Set BGR or RGB colors
        self.fontscale = LCDdata[self.disp_type]['FontScale']         # Set BGR or RGB colors

        if self.disp_rotation == 90 or self.disp_rotation == 270:      # Screen width/height is rotated
            self.Xscale = (self.disp_height / default_x )             # Scale factor is based on 160x128 screen
            self.Yscale = (self.disp_width / default_y )
        else:
            self.Xscale = (self.disp_width / default_x )              # Scale factor is based on 160x128 screen
            self.Yscale = (self.disp_height / default_y )

        if self.display_settings:
            self.disp = LCD(port=0, cs=0,                             # SPI and Chip Selection
                            dc=27, rst=22, backlight=4,               # GPIO pins used for the SPI, reset and backlight control
                            width=self.disp_width,     #(AF 132)      # see note above for width and height !!!
                            height=self.disp_height,   #(AF 162)      # see note above for width and height !!!
                            offset_left=self.disp_offsetL,            # see note above for offset  !!!
                            offset_top=self.disp_offsetT,             # see note above for offset  !!!
                            rotation=self.disp_rotation,              # image orientation
                            invert=self.disp_color_invert,            # image invertion,
                            spi_speed_hz=self.disp_spi_freq)          # SPI frequency
        
            self.disp.set_backlight(0)                                # display backlight is set off
            self.disp_w = self.disp.width                             # display width, retrieved by display setting
            self.disp_h = self.disp.height                            # display height, retrieved by display setting
            disp_img = Image.new('RGB', (self.disp_w, self.disp_h),color=self.bgr((0, 0, 0)))   # display image generation, full black
            self.disp.display(disp_img)                               # image is displayed
            if not self.display_initialized:                          # case display_initialized is set False
                print("\nDisplay initialized\n")                      # feedback is printed to the terminal
                self.display_initialized = True                       # display_initialized is set True
        
        # loading the CUBOTino logo
        folder = pathlib.Path().resolve()                             # active folder (should be home/pi/cube)
        fname = "Cubotino_T_Logo_265x212_BW.jpg"                      # file name with logo image
        fname = os.path.join(folder,fname)                            # folder and file name for the logo image
        if os.path.exists(fname):                                     # case the logo file exists
            logo = Image.open(fname)                                  # opens the CUBOTino logo image (jpg file)
            self.logo = logo.resize((self.disp_w, self.disp_h))       # resizes the image to match the display.
        else:                                                         # case the logo file does not exist
            print(f"\nNot found {fname}")                             # feedback is printedto terminal
            print("Cubotino logo image is missed\n")                  # feedback is printedto terminal
            self.logo = Image.new('RGB', (self.disp_w, self.disp_h), color=self.bgr((0, 0, 0)))  # full black screen as new image
            logo_text = ImageDraw.Draw(self.logo)                     # image is drawned
            f1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(26))  # font and size
            f2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(22))  # font and size
            logo_text.text(self.scale_xy((10, 44)), "CUBOT", font=f1, fill=self.bgr((255, 255, 255)))  # text, font, white color
            logo_text.text(self.scale_xy((112, 48)), "ino", font=f2, fill=self.bgr((255, 255, 255)))   # text, font, white color




    def bgr(self, c):                             # Function to convert colors for displays that take RGB colors
        if self.disp_colors == 'RGB':
            return ( c[2], c[1], c[0] )
        return c




    def fontsize(self, pt):                       # Change font size based on screen scaling factor
        return int(pt * self.fontscale)




    def scale_xy( self, coord):                   # Scale (x,y) coordinates based on screen scale
        scaledx = int( coord[0] * self.Xscale)
        scaledy = int( coord[1] * self.Yscale)
        return (scaledx, scaledy)




    def scale_rect( self, coord):                 # Scale (x,y,x1,y1) coordinates based on screen scale
        scaledx1 = int( coord[0] * self.Xscale)
        scaledy1 = int( coord[1] * self.Yscale)
        scaledx2 = int( coord[2] * self.Xscale)
        scaledy2 = int( coord[3] * self.Yscale)
        scaled = (scaledx1, scaledy1, scaledx2, scaledy2)
        return scaled



    def set_backlight(self, value):
        """Set the backlight on/off."""
        self.disp.set_backlight(value)




    def clean_display(self):
        """ Cleans the display by settings all pixels to black."""
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=self.bgr((0, 0, 0)))  # full black screen as new image
        self.disp.display(disp_img)                                                         # display is shown to display




    def show_on_display(self, r1,r2,x1=20,y1=25,x2=20,y2=65,fs1=22,fs2=22):
        """Shows text on two rows, with parameters to generalize this function; Parameters are
            r1, r2: text for row1 and row2
            x1, x2: x coordinate for text at row1 and row2
            y1, y2: y coordinate for text at row1 and row2
            fs1, fs2: font size for text at row1 and row2
            """
        
        font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(fs1))  # font and size for first text row
        font2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(fs2))  # font and size for second text row
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=self.bgr((0, 0, 0)))                 # full black image
        disp_draw = ImageDraw.Draw(disp_img)                                                     # image is drawned
        disp_draw.text(self.scale_xy((x1, y1)), r1, font=font1, fill=self.bgr((255, 255, 255)))    # first text row start coordinate, text, font, white color
        disp_draw.text(self.scale_xy((x2, y2)), r2, font=font2, fill=self.bgr((255, 255, 255)))    # second text row start coordinate, text, font, white color
        self.disp.display(disp_img)                                       # image is plot to the display
        self.disp.set_backlight(1)                                        # display backlight is set on




    def display_progress_bar(self, percent, scrambling=False):
        """ Function to print a progress bar on the display."""
        
        w = default_x                                              # display width, retrieved by display setting
        
        # percent value printed as text 
        fs = 40                 # font size (Scaled in next line)
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(fs))     # font and its size
        text_x = int(w/2 - (fs*len(str(percent))+1)/2)             # x coordinate for the text starting location         
        text_y = 15                                                # y coordinate for the text starting location
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=self.bgr((0, 0, 0))) # full black image
        disp_draw = ImageDraw.Draw(disp_img)                       # image is drawned
        disp_draw.text(self.scale_xy((text_x, text_y)), str(percent)+'%', font=font, fill=self.bgr((255, 255, 255)))    # text with percent value
        
        # percent value printed as progress bar filling 
        x = 15                  # x coordinate for the bar starting location
        y = 60                  # y coordinate for the bar starting location
        gap = 3                 # gap in pixels between the outer border and inner filling (even value is preferable) 
        if not scrambling:      # case the robot is solving a cube
            barWidth = 35       # width of the bar, in pixels
        elif scrambling:        # case the robot is scrambling a cube
            barWidth = 18       # width of the bar, in pixels
            fs = 18             # font size
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(fs))  # font and its size
            disp_draw.text(self.scale_xy((15, 85)), 'SCRAMBLING', font=font, fill=self.bgr((255, 255, 255)))                # SCRAMBLING text
            
        barLength = w-2*x-4 #135   # lenght of the bar, in pixels
        filledPixels = int( x+gap +(barLength-2*gap)*percent/100)  # bar filling length, as function of the percent
        disp_draw.rectangle(self.scale_rect((x, y, x+barLength, y+barWidth)), outline="white", fill=self.bgr((0,0,0)))   # outer bar border
        disp_draw.rectangle(self.scale_rect((x+gap, y+gap, filledPixels, y+barWidth-gap)), fill=self.bgr((255,255,255))) # bar filling
        
        self.disp.display(disp_img) # image is plotted to the display
        self.disp.set_backlight(1)  # display backlight is set on




    def show_cubotino(self, built_by='', x=25, fs=22):
        """ Shows the Cubotino logo on the display."""
        
        if built_by != '':                                 # case the built_by variable is not empty
            disp_draw = ImageDraw.Draw(self.logo)          # image is plotted to display
            font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(14))  # font1
            disp_draw.text(self.scale_xy((15, 10)), "Andrea FAVERO's", font=font1, fill=self.bgr((0, 0, 255)))  # first row text test
            
            font2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(10))  # font1
            disp_draw.text(self.scale_xy((60,85)), "Built by", font=font2, fill=self.bgr((255, 255, 255)))    # second row text test
            
            font3 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(fs))  # font1
            disp_draw.text(self.scale_xy((x, 104)), built_by, font=font3, fill=self.bgr((255, 0, 0)))              # third row text test
        self.disp.display(self.logo)                       # draws the image on the display hardware.
        self.disp.set_backlight(1)                         # display backlight is set on




    def show_face(self, side, colours=[]):
        """ Function to print a sketch of the cube face colours."""
        
        w = default_x                                      # display width, default size
        h = default_y                                      # display height, default size
        faces = ('', 'U', 'B', 'D', 'F', 'R', 'L')         # tuple of faces letters
        y_start = 20                                       # y coordinate for face top-left corner
        d = int(h-y_start)/3.8                             # facelet square side
        x_start = w-5-3*d                                  # x coordinate for face top-left corner
        gap = 3                                            # gap beftween the facelets border and facelets coloured part
        
        font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(17))  # font1
        font2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(70))  # font1
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=self.bgr((0, 0, 0)))  # full black image
        disp_draw = ImageDraw.Draw(disp_img)               # image is drawned
        disp_draw.text(self.scale_xy((12, 25)), 'FACE', font=font1, fill=self.bgr((255, 255, 255)))   # first row text test
        disp_draw.text(self.scale_xy((9, 40)), faces[side], font=font2, fill=self.bgr((255, 255, 255)))   # first row text test
        self.disp.set_backlight(1)                         # display backlight is set on
        
        fclt = 0
        y = y_start                                        # y coordinate value for the first 3 facelets
        for i in range(3):                                 # iteration over rows
            x = x_start                                    # x coordinate value for the first facelet
            for j in range(3):                             # iteration over columns
                disp_draw.rectangle(self.scale_rect((x, y, x+d, y+d)), outline="white", fill=self.bgr((0,0,0)))   # outer bar border
                if len(colours)==9:                        # case colour is prodided
                    disp_draw.rectangle(self.scale_rect((x+gap, y+gap, x+d-gap-1, y+d-gap-1)), self.bgr(colours[fclt])) #colours[fclt]) # bar filling
                x = x+d                                    # x coordinate is increased by square side
                if j == 2: y = y+d                         # once at the third column the row is incremented
                fclt+=1

        self.disp.display(disp_img)                        # image is plotted to the display

    
    
    def plot_status(self, cube_status, plot_color, startup=False):
        """ Function to print the cube sketch of the cube colors."""
        
        if startup:
            self.c = plot_color

            x_start = 4                                    # x coordinate for face top-left corner
            y_start = 7                                    # y coordinate for face top-left corner
            s = 2                                          # gap between faces
            self.d = 12                                    # facelet square side
            self.g = 1                                     # offset for xy origin-square of colored facelet
            self.gg = 2*self.g                             # offset for xy end-square of colored facelet
            
            # dict with the top-left coordinate of each face (not facelets !)
            starts={0:(x_start+3*self.d+  s, y_start),
                    1:(x_start+6*self.d+2*s, y_start+3*self.d+  s),
                    2:(x_start+3*self.d+  s, y_start+3*self.d+  s),
                    3:(x_start+3*self.d+  s, y_start+6*self.d+2*s),
                    4:(x_start,              y_start+3*self.d+  s),
                    5:(x_start+9*self.d+3*s, y_start+3*self.d+  s)}
            
            # coordinate origin for the 54 facelets
            tlc=[]                                         # list of all the top-left vertex coordinate for the 54 facelets
            for value in starts.values():                  # iteration over the 6 faces
                x_start=value[0]                           # x coordinate fo the face top left corner
                y_start=value[1]                           # y coordinate fo the face top left corner
                y = y_start                                # y coordinate value for the first 3 facelets
                for i in range(3):                         # iteration over rows
                    x = x_start                            # x coordinate value for the first facelet
                    for j in range(3):                     # iteration over columns
                        tlc.append((x, y))                 # x and y coordinate, as list, for the top left vertex of the facelet is appendended
                        x = x+self.d                       # x coordinate is increased by square side
                        if j == 2: y = y+self.d            # once at the second column the row is incremented
            self.tlc = tuple(tlc)                          # tlc list is converted to tuple
            
            self.disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=self.bgr((0, 0, 0)))  # full black image
            self.disp_draw = ImageDraw.Draw(self.disp_img) # image is drawned
            
        
        # below part gets updated at every new cube_status sent
        for i, color in enumerate(cube_status):            # iteration over the 54 facelets interpreted colors
            B,G,R = self.c[color]                          # BGR values of the assigned colors for the corresponding detected color
            x = self.tlc[i][0]+self.g                      # x coordinate for the origin-square colored facelet
            y = self.tlc[i][1]+self.g                      # y coordinate for the origin-square colored facelet
            dx = x + self.d - self.gg                      # x coordinate for the end-square colored facelet
            dy = y + self.d - self.gg                      # y coordinate for the end-square colored facelet
            self.disp_draw.rectangle(self.scale_rect((x, y, dx, dy)), self.bgr((B,G,R)))   # cube sketch grid
        
        self.disp.display(self.disp_img)                   # image is drawned
        self.disp.set_backlight(1)                         # display backlight is set on

    
    
    
    def test1_display(self):
        """ Test showing some text into some rectangles."""
        
        print("Display test for 20 seconds")
        print("Display shows rectangles, text and Cubotino logo")
        
        import time
        
        w = default_x                                                     # display width, scaled below
        h = default_y                                                     # display height, scaled below
        
        font1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(20))  # font1
        font2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", self.fontsize(16))  # font2
        disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=self.bgr((0, 0, 0)))      # full black image
        
        self.disp.set_backlight(1)                                        # display backlight is set on
        start = time.time()                                               # time refrence for countdown
        timeout = 20.5                                                    # timeout
        while time.time() < start + timeout:                              # while loop until timeout
            
            t_left = int(round(timeout + start - time.time(),0))          # time left
            
            if t_left%2==0:                                               # case the time left is even
                t_left_str = str(t_left)                                  # string of time left
                pos = w-40 if t_left>9 else w-35                          # position x for timeout text
                disp_img = Image.new('RGB', (self.disp_w, self.disp_h), color=self.bgr((0, 0, 0)))      # full black image
                disp_draw = ImageDraw.Draw(disp_img)                      # image is drawned

                disp_draw.rectangle(self.scale_rect((2, 2, w-4, h-4)), outline="white", fill=self.bgr((0,0,0)))    # border 1
                disp_draw.rectangle(self.scale_rect((5, 5, w-7, h-7)), outline="white", fill=self.bgr((0,0,0)))    # border 2
                disp_draw.rectangle(self.scale_rect((8, 8, w-10, h-10)), outline="white", fill=self.bgr((0,0,0)))  # border 3
                disp_draw.rectangle(self.scale_rect((w-45, h-40, w-14, h-14)), outline="blue", fill=self.bgr((0,0,0))) # border for timeout
                
                disp_draw.text(self.scale_xy((pos, h-36)), t_left_str , font=font2, fill=self.bgr((0, 0, 255)))  # timeout text
                disp_draw.text(self.scale_xy((30, 25)), 'DISPLAY', font=font1, fill=self.bgr((255, 255, 255)))   # first row text test
                disp_draw.text(self.scale_xy((33, 75)), 'TEST', font=font1, fill=self.bgr((255, 255, 255)))      # second row text test
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

    
