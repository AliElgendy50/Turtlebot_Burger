#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt

class DifferentialDriveRobot:
    def __init__(self, wheel_radius, wheel_base):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.x = 0.0 # initial x position
        self.y = 0.0 # initial y position
        self.theta = 0.0 # initial orientation
        self.vx = 0.0 # initial linear velocity
        self.vtheta = 0.0 # initial angular velocity
        self.history = {'x': [], 'y': [], 'theta': []}

    def odom_callback(self, msg):
        # Extract pose information from Odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        # Extract linear and angular velocities
        self.vx = msg.twist.twist.linear.x
        self.vtheta = msg.twist.twist.angular.z

        # Append current pose to history
        self.history['x'].append(self.x)
        self.history['y'].append(self.y)
        self.history['theta'].append(self.theta)

    def plot_robot(self):
        # Plot robot
        plt.plot(self.x, self.y, 'ro')
        plt.quiver(self.x, self.y, np.cos(self.theta), np.sin(self.theta))
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Differential Drive Robot Simulation')
        plt.axis('equal')
        plt.grid(True)

def main():
    # Initialize ROS node
    rospy.init_node('robot_visualization', anonymous=True)

    # Create instance of DifferentialDriveRobot class
    robot = DifferentialDriveRobot(wheel_radius=0.05, wheel_base=0.2)

    # Subscribe to the Odometry topic published by robot state publisher
    rospy.Subscriber('/odom', Odometry, robot.odom_callback)

    # Simulation loop
    while not rospy.is_shutdown():
        robot.plot_robot()
        plt.pause(0.1)

    plt.show()

if __name__ == '__main__':
    main()
