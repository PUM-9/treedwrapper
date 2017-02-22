#!/usr/bin/env python

import rospy
import os
from treedwrapper.srv import Scan

def wrapper_scan_test():
    rospy.wait_for_service('scan')
    scan = rospy.ServiceProxy('scan', Scan)
    try:
        res = scan(50, 50)
    except rospy.ServiceException:
        print("Service error")
    print("hej")
    print(res)
         
if __name__ == "__main__":
    wrapper_scan_test()
    
