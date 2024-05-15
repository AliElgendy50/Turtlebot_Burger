#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class ObstacleAvoidanceController:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_control')

        # Subscriber for object position
        rospy.Subscriber('/object_position', Point, self.object_position_callback)

        # Publisher for motion commands
        self.motion_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

        # Create an instance of Twist message
        self.motion_cmd = Twist()

        # Initialize variables for object position
        self.object_x = 0
        self.object_y = 0

        rospy.spin()

    def object_position_callback(self, msg):
        # Update object position
        self.object_x = msg.x
        self.object_y = msg.y

        # Generate motion commands based on object position
        self.generate_motion_commands()

    def generate_motion_commands(self):
        if self.object_x != 0 and self.object_y != 0:
            # Implement logic to bypass the object based on its position
            if self.object_x > 560 or self.object_y>=347:  # Assuming 640x480 frame, 320 is the center
                # object is on the left, so go right to bypass it
                self.motion_cmd.linear.x = 3
                self.motion_cmd.angular.z = -40  # Turn right
            # else:
            #     # object is on the right, so go left to bypass it
            #     self.motion_cmd.linear.x = 3
            #     self.motion_cmd.angular.z = 60   # Turn left
        else:
            # If object position is not detected, maintain forward motion
            self.motion_cmd.linear.x = 4  # Default linear velocity
            self.motion_cmd.angular.z = 0.0  # Default angular velocity

        # Publish motion commands
        self.motion_pub.publish(self.motion_cmd)

if __name__ == '__main__':
    try:
        ObstacleAvoidanceController()
    except rospy.ROSInterruptException:
        pass
