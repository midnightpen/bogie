#!/usr/bin/env python
# -*- coding: utf-8 -*-
#sudo -H pip install luma.core==1.13.0 luma.led_matrix==1.4.1

import rospy
from std_msgs.msg import String

import time #, json, urllib.request
import luma.core.error
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
# from luma.core.legacy import show_message
# from luma.core.legacy.font import proportional, CP437_FONT
from luma.core.render import canvas
# Offset values (in pixels)
offset_x = 1
offset_y = -2
text_to_display= ""
def string_callback(msg):
    global text_to_display
    text_to_display = msg.data

def led_display():
    global text_to_display

    serial = spi(port=1, device=0, gpio=noop())
    device = max7219(serial, width=32, height=8, block_orientation=-90, rotate=0)
    # print(text_to_display)
    if text_to_display is not "":
        with canvas(device) as draw:
        # You can set the position and style of the text here
            draw.text((offset_x, offset_y), text_to_display, fill="white")
        time.sleep(5)

    r.sleep()

    

if __name__ == "__main__":
    rospy.init_node('LED_display')
    rospy.Subscriber('/qr_data', String, string_callback)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        led_display()


  

