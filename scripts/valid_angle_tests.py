#!/usr/bin/env python
"""
Tests that the x_angle and y_angle validation is done correctly.
This is done by using boundary testing with sample input and
expected output.
"""
from main import is_valid_x_angle, is_valid_y_angle

if __name__ == "__main__":

    # x_angle in data and expected output
    x_test_input = [-19, -20, -21, 30, 89, 90, 91]
    x_test_expected_output = [True, True, False, True, True, True, False]

    # y_angle in data and expected output
    y_test_input = [-1, 0, 1, 120, 358, 359, 360]
    y_test_expected_output = [False, True, True, True, True, True, False]

    failed = False

    # Test x_angle validation
    for i in range(0, len(x_test_input)):
        if not x_test_expected_output[i] == is_valid_x_angle(x_test_input[i]):
            print "Test failed on x_angle: %d" % x_test_input[i]
            failed = True

    # Test y_angle validation
    for i in range(0, len(y_test_input)):
        if not y_test_expected_output[i] == is_valid_y_angle(y_test_input[i]):
            print "Test failed on y_angle: %d" % y_test_input[i]
            failed = True

    if not failed:
        print "Angle test passed"
