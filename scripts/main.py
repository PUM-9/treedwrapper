#!/usr/bin/env python

import rospy
import os
<<<<<<< HEAD
from treedwrapper.srv import Scan, ScanResponse
=======
from treedwrapper.srv import Scan
>>>>>>> github/master


def wrapper_scan(req):
    """
    Handles a scan request, by completing a scan at defined angles and returns a PCL object. Allowed angles are for
    y_angle 0 to 359 and x_angle -20 to 90.
    :param req: (ROS request) The request with defined angles
    :return: (ROS response) Response with pcl object if everything went ok otherwise exit code 1 and error message.
    """
    # Check if the angles are allowed.
    if (req.y_angle < 0 or req.y_angle > 359) or (req.x_angle < -20 or req.x_angle > 90):
        return ScanResponse(None, 1, "Angle is not allowed")
    os.system("treed set --table-rotation " + str(req.y_angle))
    os.system("treed set --table-curve " + str(req.x_angle))
    os.system("treed scan -o /tmp/recentscan.pcd")
    # TODO: Read file to pcl object and return it.
    return ScanResponse(None, 0, "")


def main():
    """
    Spins up the scan service.
    :return: None
    """
    rospy.init_node('treedwrapper', anonymous=False)
    scan_service = rospy.Service('scan', Scan, wrapper_scan)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
