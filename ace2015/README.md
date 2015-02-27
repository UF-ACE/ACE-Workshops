Contents
========

src/PublisherDemo.py
  - Basic Publisher demo source code from the ROS tutorials
  - Excellent skeleton code for writing your own Publisher
  
src/PyGameDemo.py
  - Source code for the "PyGameDemo" ROS node. Handles the PyGame drawing and the image loading, etc
  
src/Robot.py
  - Simple container class for the robot sprite
  
src/Util.py
  - Contains a load_image method 
  
src/RobotController.py
  - Source code for the "RobotController" ROS node. Handles the logic behind moving the robot around the screen.
  - Subscribers to the current position and desired position messages in order to calculate the next action for the bot. 

After moving the ace2015 folder into your catkin workspace's src folder, run "catkin_make" in the root of your workspace to build. You should then be able to run the application with $ roslaunch ace2015 PyGameDemo.launch
