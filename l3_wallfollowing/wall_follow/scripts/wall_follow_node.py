#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # create subscribers and publishers
        self.sub_lidar = self.create_subscription(LaserScan,lidarscan_topic, self.scan_callback, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # set PID gains
        self.kp = 1.0
        self.kd = 0.0
        self.ki = 0.0

        # store history
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        # tune-able hyperparameters
        self.steering_angle = 0.0
        self.desired_dist = 1.2
        self.angleA = 135 + 90 - 45
        self.angleB = 135 + 90
        self.theta  = np.radians(np.abs(self.angleB - self.angleA))
        self.L      = 0.8

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            distance: range measurement in meters at the given angle

        """
        # get range measurement corresponding to given angle
        index = int(angle * (len(range_data) / 270.0))
        distance = range_data[index]

        # replace NaNs and infs using neighbor scan measurements
        neighbors = [index - 1, index + 1, index - 2, index + 2, index - 3, index + 3]
        for i in neighbors:
            if not (np.isnan(distance) or np.isinf(distance)):
                break
            self.get_logger().info('[get_range] Invalid range measurement.')
            distance = range_data[i]

        # if neighbor scan measurements are also invalid
        if np.isinf(distance):
            distance = 50
        elif np.isnan(distance):
            distance = 0

        return distance

    def get_error(self, range_data, desired_dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            desired_dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        a = self.get_range(range_data, self.angleA)
        b = self.get_range(range_data, self.angleB)

        alpha = np.arctan2(a * np.cos(self.theta) - b, a * np.sin(self.theta))
        Dt    = (b * np.cos(alpha)) + (self.L * np.sin(alpha))

        return desired_dist - Dt

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # use kp, ki, and kd to implement PID controller
        self.steering_angle = -(self.kp * error) + \
                               (self.ki * self.integral) + \
                               (self.kd * (error - self.prev_error))

        # fill in drive message and publish
        drive_msg                      = AckermannDriveStamped()
        drive_msg.drive.speed          = velocity
        drive_msg.drive.steering_angle = self.steering_angle
        self.pub_drive.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """        
         # error calculated by get_error()
        self.error = self.get_error(msg.ranges, self.desired_dist)
        
        # calculate desired car velocity based on error
        velocity = 0.5                  
        if(np.abs(self.steering_angle) < 10):
            velocity = 1.5
        elif(np.abs(self.steering_angle) < 20):
            velocity = 1.0

        # actuate the car with PID
        self.pid_control(self.error, velocity)

        # update Errors
        self.integral  += self.error
        self.prev_error = self.error


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
