#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import rclpy

class Turtlebot3ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')

        # Initialize the variables
        self.linear_velocity = 0.25  # unit: m/s
        self.angular_velocity = 0.0  # unit: m/s
        self.scan_ranges = []
        self.init_scan_state = False  # To get the initial scan data at the beginning
        self.new_obstacle_detected = False
        self.angular_speed = 0.5

        """---------------Initialise ROS publishers and subscribers------------------"""
        # Set the QoS profile for the publisher and subscriber
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        # Create a subscriber to the scan topic which is of type LaserScan and calls the scan_callback function
        self.scan_sub = self.create_subscription(LaserScan,'scan',self.scan_callback,qos_profile=qos_profile_sensor_data)
        
        # Create a subscriber to the cmd_vel topic which is of type Twist and calls the cmd_vel_raw_callback function
        self.cmd_vel_raw_sub = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_raw_callback,qos)

        # Create a timer to update the robot's movement every 0.01 seconds
        self.update_timer = self.create_timer(0.010, self.update_callback)

        # Print a message to the console
        self.get_logger().info("Turtlebot3 obstacle detection node has been initialised.")

    """--------------Callback functions and relevant functions----------------"""

    # Callback function for the scan subscriber
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.init_scan_state = True

    # Callback function for the cmd_vel_raw subscriber
    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        #print("Cmd_vel_raw_callback check")

    # Callback function for the update timer
    def update_callback(self):
        if self.init_scan_state is True:
            self.detect_obstacle()

    #Function to detect the obstacle and avoid it
    def detect_obstacle(self):
        twist = Twist()
        self.get_logger().info(f'Lidar Scan Ranges: {self.scan_ranges}')
        thresh1 = 0.7 # Laser scan range threshold
        thresh2 = 0.8
        min_right_range = min(self.scan_ranges[0:20])
        min_left_range = min(self.scan_ranges[340:-1])
        if min_right_range > thresh1 and min_left_range > thresh1:
            self.new_obstacle_detected = False
            twist.linear.x = 0.25
            twist.angular.z = 0.0
        else:
            if not self.new_obstacle_detected:
                if min(self.scan_ranges[0:60]) > min(self.scan_ranges[300:-1]):   #Rotate Right
                    self.angular_speed = 0.5
                else:                                  #Rotate Left
                    self.angular_speed = -0.5
            twist.linear.x = 0.0  
            twist.angular.z = self.angular_speed 
            self.new_obstacle_detected = True
            self.get_logger().info(f'Obstacle Detected: {self.new_obstacle_detected}')
        self.cmd_vel_pub.publish(twist)

        
# Main function        
def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleAvoidance()
    rclpy.spin(turtlebot3_obstacle_detection)
    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
