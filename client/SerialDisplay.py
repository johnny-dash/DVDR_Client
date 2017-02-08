#!/usr/bin/env python

# 

from grove_rgb_lcd import *
from GrovepiSerial import getserial

def serialDisplay():
    serial = getserial()
    setRGB(0,255,0)
    setText("Serial Number:  %s" % serial)
