#!/usr/bin/python
# coding: utf-8

"""
#############################################################################################################
#  Andrea Favero 25 October 2026
#
# This script relates to CUBOTino autonomous, a small and simple Rubik's cube solver robot 3D printed
# This specific script checks the pigpiod status, and reset it once at the start
# This is a workaround, to solve some pigpiod issues when automatically loaded at Raspberry Pi boot
#
# on os <= 11 the daemon is killed and restarted once per script run, as a
# workaround for a boot-time race that causes the display backlight pwm and the
# servo pwm to share resources (as per the original Cubotino_T_pigpiod.py).
#
# on os >= 12 the daemon is started only if not already reachable, since it is
# not shipped by the distro and restarting it has a different cost profile.
#############################################################################################################
"""

import Cubotino_T_platform_io as platform_io

class Pigpiod:
    pigpiod_once = False
    
    def __init__(self):
        if not self.pigpiod_once:
            ok = platform_io.ensure_pigpiod(verbose=True)
            if ok:
                self.pigpiod_once = True

pigpiod = Pigpiod()


if __name__ == "__main__":
    from Cubotino_T_display import display as s_disp
    s_disp.test1_display()
    

