#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

class RobotController(object): 

    def __init__(self): 
        # initiliaze desired position and current position to blank PointStamped messages
        self.cur_pos = PointStamped()
        self.des_pos = PointStamped()

        # Create our Publisher, which will publish the /delta_pos topic
        self.delta_pos = rospy.Publisher("/delta_pos", PointStamped, queue_size=1)

        # Set our subscribers, which subscribe to the /cur_pos and /des_pos topics
        rospy.Subscriber("/cur_pos", PointStamped, self.cur_pos_callback)
        rospy.Subscriber("/des_pos", PointStamped, self.des_pos_callback)

    # Current Position callback function. The data is a full PointStamped message
    def cur_pos_callback(self, data):
        # rospy.loginfo("Updated current position %s", str(data))
        self.cur_pos = data

    # Desired position callback function
    def des_pos_callback(self, data): 
        # rospy.loginfo("Updated desired position %s", str(data))
        self.des_pos = data

    def publish_delta_pos(self, pos):
        """
        Publishes the ~change~ in the x and y coordinates of the robot
        """
        self.delta_pos.publish(
                    PointStamped(
                        header = Header(
                                stamp=rospy.Time.now(),
                                frame_id='/robot',
                        ),
                        point=Point(x=pos[0],
                                    y=pos[1],
                                    z=0
                        )
                    )
                )

    def calculate_delta(self):
        """
        Calculates the change in x and y needed to move closer to the 
        desired position from the current position
        """
        if self.des_pos.point.x > self.cur_pos.point.x:
            # move by five pixels at a time
            delta_pos_x = 5
        elif self.des_pos.point.x < self.cur_pos.point.x:
            delta_pos_x = -5
        else:
            delta_pos_x = 0

        if self.des_pos.point.y > self.cur_pos.point.y:
            # move by five pixels at a time
            delta_pos_y = 5
        elif self.des_pos.point.y < self.cur_pos.point.y:
            delta_pos_y = -5
        else:
            delta_pos_y = 0

        return (delta_pos_x, delta_pos_y)


    def compare_pos(self): 
        """
        Contains a threshold criterion to compare two points and consider if they are the same or not
        """
        if np.absolute(self.des_pos.point.x - self.cur_pos.point.x) > 5:
            return True
        elif np.absolute(self.des_pos.point.y - self.cur_pos.point.y) > 5: 
            return True
        else: 
            return False

    def controller(self):

        # Init my ROS Node
        rospy.init_node('RobotController', anonymous=True)

        # The ROS rate is set to 10 Hz
        rate = rospy.Rate(10)

        while not rospy.is_shutdown(): 
            # Check if the current position and desired position are not the same
            if self.compare_pos():  
                # Calculate the necessary change in position and publish it as /delta_pos
                # rospy.loginfo("delta position is %s", str(self.calculate_delta()))
                self.publish_delta_pos(self.calculate_delta())

            # sleep the process
            rate.sleep()

if __name__ == '__main__':
    try:
        robot = RobotController()
        robot.controller()
    except rospy.ROSInterruptException: 
        pass
