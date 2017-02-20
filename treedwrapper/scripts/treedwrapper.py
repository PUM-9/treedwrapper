#!/usr/bin/env python

import rospy
import os

def wrapper_scan(rot, cur):
    if((rot < 0 or rot > 359) or (cur < -20 or cur > 90)):
        return
    os.system("treed set --table-rotation " + str(rot))
    os.system("treed set --table-curve " + str(cur))
    os.system("treed scan -o recentscan.pcd")
    
if __name__ == '__main__':
    wrapper_scan(0,0)
