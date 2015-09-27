#!/usr/bin/env python

##########################################################################
# Copyright 2015 Sensel, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
##########################################################################

#
#  Read Contacts
#  by Aaron Zarraga - Sensel, Inc
# 
#  This opens a Sensel sensor, reads contact data, and prints the data to the console.
#
#  Note: We have to use \r\n explicitly for print endings because the keyboard reading code
#        needs to set the terminal to "raw" mode.
##

from __future__ import print_function
from keyboard_reader import *
import sensel
import time
import libardrone

exit_requested = False;

def keypress_handler(ch):
    global exit_requested

    if exit_requested:
        exit()

    if ch == 0x51 or ch == 0x71: #'Q' or 'q'
        print("Exiting Example...", end="\r\n");
        exit_requested = True;


def openSensorReadContacts():
    sensel_device = sensel.SenselDevice()

    if not sensel_device.openConnection():
        print("Unable to open Sensel sensor!", end="\r\n")
        exit()

    keyboardReadThreadStart(keypress_handler)

    #Enable contact sending
    sensel_device.setFrameContentControl(sensel.SENSEL_FRAME_CONTACTS_FLAG)
  
    #Enable scanning
    sensel_device.startScanning()

    print("\r\nTouch sensor! (press 'q' to quit)...", end="\r\n")

    init_contact_uid = 0
    ticks_hovering = time.time()
    init_x = 0 
    init_y = 0 

    drone = libardrone.ARDrone()
    drone.takeoff()
    time.sleep(3)

    while not exit_requested: 
        contacts = sensel_device.readContacts()
  
        if len(contacts) == 0:
            init_x = 0 
            init_y = 0 
            drone.hover()
            print("hovering")
            continue
        elif len(contacts) == 1:
            if init_x is 0 and init_y is 0:
                init_x = contacts[0].x_pos_mm
                init_y = contacts[0].y_pos_mm
            elif contacts[0].total_force >= 2500 and \
                time.time() - ticks_hovering >= 2:
                drone.move_up()
                print("moving up")
                continue
        elif len(contacts) == 2:
            if (contacts[0].total_force + contacts[1].total_force) / 2 >= 2500 and \
                time.time() - ticks_hovering >= 2:
                drone.move_down()
                print("moving down")
                continue

        if not(init_y is 0 or init_x is 0):
            this_x = contacts[0].x_pos_mm
            this_y = contacts[0].y_pos_mm

            diff_x = this_x - init_x
            diff_y = this_y - init_y

            v_x = diff_x / 120.0
            v_y = diff_y / 120.0

            v_x = max(v_x, -.5)
            v_x = min(v_x, .5)
            v_y = max(v_y, -.5)
            v_y = min(v_y, .5)

            drone.set_v_x(v_x)
            drone.set_v_y(v_y)

            if abs(v_x) < .1 and abs(v_y) < .1:
                drone.hover()
                print("hovering")
            else:
                if abs(v_x) > abs(v_y):
                    drone.move_right()
                else:
                    drone.move_forward()
                print("velocity: ({}, {})".format(v_x, v_y))

        else:
            drone.set_v_x(0)
            drone.set_v_y(0)
            drone.hover()
            print("hovering")

    sensel_device.stopScanning();
    sensel_device.closeConnection();
    keyboardReadThreadStop()

if __name__ == "__main__":
    openSensorReadContacts()
    
