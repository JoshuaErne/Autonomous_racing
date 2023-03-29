#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Subscriber to LIDAR topic
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

        # Publisher to DRIVE topic
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # Parameters
        self.filter_width     = 4
        self.dist_thresh      = 1.52
        self.gap_width_thresh = 40

    def preprocess_lidar(self, ranges):
        """
        Preprocess the LiDAR scan array. Expert implementation includes:
            1. Setting each value to the mean over some window
            2. Rejecting high values (eg. > 3m)
        """

        # handle nan and inf values
        proc_ranges = np.nan_to_num(ranges)

        # reject high range measurements (> 3m)
        # proc_ranges = np.clip(proc_ranges, 0, 10)
        
        # filter over moving window
        # proc_ranges = np.convolve(proc_ranges, np.ones(self.filter_width)/self.filter_width, mode='same')

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """
        Return the start & end index of the max gap in free_space_ranges
        """
        gaps = []
        start_i, end_i = 0, 0
        best_start, best_end = 0, 0

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > self.dist_thresh:
                end_i += 1
            else:
                if end_i >= start_i + self.gap_width_thresh:
                    gaps.append((start_i, end_i))

                if end_i - start_i > best_end - best_start:
                    best_start, best_end = start_i, end_i
                
                start_i = i + 1
                end_i = i

        if end_i - start_i > best_end - best_start:
            best_start = start_i
            best_end   = end_i

        best_mean_depth = np.mean(free_space_ranges[best_start:best_end+1])
        best_indices = (best_start, best_end)
        for start, end in gaps:
            mean_depth = np.mean(free_space_ranges[start:end+1])
            if (mean_depth > best_mean_depth):
                best_mean_depth = mean_depth
                best_indices = (start, end)

        return best_indices[0], best_indices[1]
    
    def find_best_point(self, start_i, end_i, ranges):
        """
        start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the center point of the gap
        """
        return (start_i + end_i) // 2

    def lidar_callback(self, data):
        """
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        # Pre-process LiDAR scan range data
        proc_ranges = self.preprocess_lidar(data.ranges)

        # Find closest LiDAR point to car
        min_index = np.argmin(proc_ranges)
        min_range = np.min(proc_ranges)

        # Eliminate all points inside 'bubble' (set them to zero) 
        if min_index > np.radians(45) / data.angle_increment and min_index < np.radians(270 - 45) / data.angle_increment:
            R          = min_range
            arc_length = 0.1                                            # size of car
            theta      = (360 * arc_length)/(2 * np.pi * R + 0.00001)   # degrees
            rb         = int(np.radians(theta) / (data.angle_increment))
            proc_ranges[max(min_index - rb, 0) : min(min_index + rb, 1079)] = 0
        
        # Find max length gap 
        start_best, end_best = self.find_max_gap(proc_ranges)

        # Check if max gap is above threshold
        if end_best >= (start_best + self.gap_width_thresh):

            # Find the best point in the gap 
            best_point = self.find_best_point(start_best, end_best, proc_ranges)

            # Determine steering angle
            steering_angle = best_point * data.angle_increment + data.angle_min

            # Determine velocity
            velocity = 1.5
            if (np.abs(steering_angle) < np.radians(10)):
                velocity = 3.0  
            elif (np.abs(steering_angle) < np.radians(20)):
                velocity = 1.5

            # Publish drive message
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed          = 2.5 #velocity
            drive_msg.drive.steering_angle = steering_angle
            self.drive_pub.publish(drive_msg)

            print(np.degrees(start_best * data.angle_increment), np.degrees(end_best * data.angle_increment), best_point, np.degrees(steering_angle))



def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
