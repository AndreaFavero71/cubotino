#!/usr/bin/python
# coding: utf-8

# from https://gist.github.com/rwb27/a23808e9f4008b48de95692a38ddaa08/
# changed part of the variables named "cam" in "camera", otherwise some errorr were returned

from __future__ import print_function
import picamera
from picamera import mmal, mmalobj, exc
from picamera.mmalobj import to_rational
import time


MMAL_PARAMETER_ANALOG_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x59
MMAL_PARAMETER_DIGITAL_GAIN = mmal.MMAL_PARAMETER_GROUP_CAMERA + 0x5A

def set_gain(camera, gain, value):
    """Set the analog gain of a PiCamera.
    
    camera: the picamera.PiCamera() instance you are configuring
    gain: either MMAL_PARAMETER_ANALOG_GAIN or MMAL_PARAMETER_DIGITAL_GAIN
    value: a numeric value that can be converted to a rational number.
    """
    if gain not in [MMAL_PARAMETER_ANALOG_GAIN, MMAL_PARAMETER_DIGITAL_GAIN]:
        raise ValueError("The gain parameter was not valid")
    ret = mmal.mmal_port_parameter_set_rational(camera._camera.control._port, 
                                                    gain, to_rational(value))
    if ret == 4:
        raise exc.PiCameraMMALError(ret, "Are you running the latest version of the userland libraries? Gain setting was introduced in late 2017.")
    elif ret != 0:
        raise exc.PiCameraMMALError(ret)

def set_analog_gain(camera, value):
    """Set the gain of a PiCamera object to a given value."""
    set_gain(camera, MMAL_PARAMETER_ANALOG_GAIN, value)

def set_digital_gain(camera, value):
    """Set the digital gain of a PiCamera object to a given value."""
    set_gain(camera, MMAL_PARAMETER_DIGITAL_GAIN, value)


if __name__ == "__main__":
    with picamera.PiCamera() as camera:
        camera.start_preview(fullscreen=False, window=(0,50,640,480))
        time.sleep(2)

        # fix the auto white balance gains at their current values
        g = camera.awb_gains
        camera.awb_mode = "off"
        camera.awb_gains = g

        # fix the shutter speed
        camera.shutter_speed = camera.exposure_speed

        print("Current a/d gains: {}, {}".format(camera.analog_gain, camera.digital_gain))

        print("Attempting to set analogue gain to 1")
        set_analog_gain(camera, 1)
        print("Attempting to set digital gain to 1")
        set_digital_gain(camera, 1)


        try:
            while True:
                print("Current a/d gains: {}, {}".format(camera.analog_gain, camera.digital_gain))
                time.sleep(1)
        except KeyboardInterrupt:
            print("Stopping...")
