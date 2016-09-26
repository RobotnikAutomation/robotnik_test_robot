# robotnik_test_robot

This package helps the task of test movility of robots.

Modify as you wish the launch and config files for the specs of your robot.

Before run the test.launch, you must reset the odom to x=0 and y=0

The test_square node will move the robot throught various points doing a square.

C---B


D---A
 
The robot will start at point A.

IMPORTANT: Edit bringup/config/costmap_common_params.yaml to specify your obstacle detection layer.
