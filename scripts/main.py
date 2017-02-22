#!/usr/bin/env python

import rospy
import os
import pcl
import time
from treedwrapper.srv import WrapperScan, WrapperScanResponse
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from watchdog import Watchdog, HardwareError

def wrapper_scan(req):
    """
    Handles a scan request, by completing a scan at defined angles and returns a PCL object. Allowed angles are for
    y_angle 0 to 359 and x_angle -20 to 90.
    :param req: (ROS request) The request with defined angles
    :return: (ROS response) Response with pcl object if everything went ok otherwise exit code 1 and error message.
    """

    x = req.x_angle
    y = req.y_angle
    default_file_path = "/tmp/recentscan.pcd"

    # Check if the angles are allowed.
    if (is_valid_angle(x, y)):
        return WrapperScanResponse(None, 1, "Angle is not allowed")

    
    try:
        watchdog_scan = Watchdog(5)
        os.system("treed set --table-rotation " + str(y))
        os.system("treed set --table-curve " + str(x))
        os.system("treed scan -o " + default_file_path)
    except:
        print "except"
        return WrapperScanResponse(None, 1, "has")
        
    watchdog_scan.stop()

	# Load in all the gathered points into a numpy array
    p = pcl.load(default_file_path)

	# Convert points gathered from scan to PointCloud2 object.
    pcloud = PointCloud2()
    pcloud = pc2.create_cloud_xyz32(pcloud.header, p.to_list())

    return WrapperScanResponse(pcloud, 0, "")

def is_valid_angle(x, y):
    """
    Checks whether the angle for rotating the object (y) and the angle for rotating the
    table (x) upward/downward is allowed.
    :param x: -20 to 90
    :param y: 0 to 359
    :return: bool Whether
    """

    return (x < -20 or x > 90) or (y < 0 or y > 359)

def main():
    """
    Spins up the wscan service.
    :return: None
    """
    rospy.init_node('treedwrapper', anonymous=False)
    wrapper_scan_service = rospy.Service('wrapperscan', WrapperScan, wrapper_scan)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
