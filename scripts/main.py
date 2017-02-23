#!/usr/bin/env python

import rospy
import os
import pcl
from treedwrapper.srv import WrapperScan, WrapperScanResponse
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import pexpect


def is_valid_x_angle(x):
    """
    Check if the angle for curve the table (x) upward/downward is allowed.
    :param x: -20 to 90
    :return: bool, true if allowed, false if not allowed.
    """
    return -20 <= x <= 90


def is_valid_y_angle(y):
    """
    Check if the angle for rotating the board (y) is allowed.
    :param y: 0 to 359
    :return: bool, true if allowed, false if not allowed.
    """
    return 0 <= y <= 359


def wrapper_scan(req):
    """
    Handles a scan request, by completing a scan at defined angles and returns a PCL object. Allowed angles are for
    y_angle 0 to 359 and x_angle -20 to 90.
    :param req: (ROS request) The request with defined angles
    :return: (ROS response) Response with pcl object if everything went ok otherwise exit code 1 and error message.
    """
    x = req.x_angle
    y = req.y_angle
    default_file_path = "/tmp/recent_scan.pcd"

    # Check if the X angle is allowed.
    if not is_valid_x_angle(x):
        return WrapperScanResponse(None, 1, "X value is not allowed, should be between 0 and 359")

    # Check if the Y angle is allowed.
    if not is_valid_y_angle(y):
        return WrapperScanResponse(None, 1, "Y value is not allowed, should be between -20 and 90")
        
    # Rotate the object and table.
    os.system("treed set --table-rotation " + str(y))
    os.system("treed set --table-curve " + str(x))
    
    # Confirm that a scan would be successfully completed.
    try:
        # Starting a child process running the scan command.
        child_process = pexpect.spawn("treed scan -o " + default_file_path)
        # Timeouts after 40 seconds.
        child_process.expect('File saved to ' + default_file_path, timeout=40)
    except pexpect.TIMEOUT:
        return WrapperScanResponse(None, 1, "There is something wrong with the hardware")

    # Load in all the gathered points into a numpy array.
    points = pcl.load(default_file_path)

    # Convert points gathered from scan to PointCloud2 object.
    point_cloud = PointCloud2()
    point_cloud = pc2.create_cloud_xyz32(point_cloud.header, points.to_list())

    return WrapperScanResponse(point_cloud, 0, "")


def main():
    """
    Spins up the WrapperScan service.
    :return: None
    """
    rospy.init_node('treed_wrapper', anonymous=False)
    rospy.Service('wrapper_scan', WrapperScan, wrapper_scan)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
