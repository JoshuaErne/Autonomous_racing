#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.frame_id = 0

        # hyper-parameters
        self.L = 1.7
        self.segments = 1024
        self.is_sim = True
        self.path_to_waypoints = "src/lab6/waypoints.csv"

        # create ROS subscribers and publishers
        if self.is_sim:
            self.pose_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.pose_callback, 10)
        else:
            self.pose_sub = self.create_subscription(PoseStamped, "/pf/viz/inferred_pose", self.pose_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/pf/viz/inferred_pose", self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)
        self.marker_hist_pub = self.create_publisher(Marker, "/visualization_marker_history", 10)

        # import saved waypoints
        self.waypoints = self.interpolate_waypoints(
            file_path=self.path_to_waypoints,
            segments=self.segments
        )
        self.goal_point = self.waypoints[0]

    def interpolate_waypoints(self, file_path, segments):
        # Read waypoints from csv
        points = np.genfromtxt(file_path, delimiter=",")[:, :2]

        # Add first point as last point to complete loop
        points = np.vstack((points, points[0]))

        # Linear length along the line
        distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]

        # Interpolate
        alpha = np.linspace(0, 1, segments)
        interpolator = interp1d(distance, points, kind='slinear', axis=0)
        interpolated_points = interpolator(alpha)

        # Add z-coordinate to be 0
        interpolated_points = np.hstack(
            (interpolated_points, np.zeros((interpolated_points.shape[0], 1)))
        )

        return interpolated_points

    def transform_waypoints(self, waypoints, car_position, pose):
        # translation
        waypoints = waypoints - car_position

        # rotation
        quaternion = np.array([
            pose.orientation.x, 
            pose.orientation.y, 
            pose.orientation.z, 
            pose.orientation.w
        ])
        waypoints = R.inv(R.from_quat(quaternion)).apply(waypoints)

        return waypoints

    def determine_velocity(self, angle):
        velocity = 2.0
        if (np.abs(angle) < np.radians(10)):
            velocity = 3.0
        elif (np.abs(angle) < np.radians(20)):
            velocity = 2.5
        return velocity

    def pose_callback(self, pose_msg):
        pose = pose_msg.pose
        if self.is_sim:
            pose = pose_msg.pose.pose
           
        # get current position of car
        x, y = pose.position.x, pose.position.y
        position = (x, y, 0)

        if (x > -3.0) and (x < 3.0) and y > -2.0 and y < 1.0:
            self.L = 1.3
        elif (x > 1.0) and (x < 3.0) and y > 5.0 and y < 8.0:
            self.L = 1.3
        elif (x < -15.0) and (x > -22.0) and y > 8.0 and y < 10.0:
            self.L = 1.3
        elif x > -22.0 and x < -18.0 and y > -1.0 and y < 3.5:
            self.L = 1.3
        else:
            self.L = 4.0

        # transform way-points from world to vehicle frame of reference
        waypoints = self.transform_waypoints(self.waypoints, position, pose)

        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints, axis=1)

        # get indices of waypoints that are within L, sorted by descending distance
        indices_L = np.argsort(np.where(distances < self.L, distances, -1))[::-1]

        # set goal point to be the farthest valid waypoint within distance L
        goal_point_world = None
        for i in indices_L:
            # check waypoint is in front of car
            x = waypoints[i][0]
            if x > 0:
                self.goal_point = waypoints[i]
                goal_point_world = self.waypoints[i]
                break

        # calculate curvature/steering angle
        L = np.linalg.norm(self.goal_point)
        y = self.goal_point[1]
        angle = (2 * y) / (L**2)
        angle = np.clip(angle, -0.3, 0.3)
        velocity = self.determine_velocity(angle)

        # publish waypoint marker
        marker = Marker()
        marker.header.frame_id = pose_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = goal_point_world[0]
        marker.pose.position.y = goal_point_world[1]
        marker.pose.position.z = 0.0
        self.marker_pub.publish(marker)

        # publish waypoint marker history
        marker.color.a = 0.5
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.id = self.frame_id + 1
        marker.lifetime = Duration(seconds=5).to_msg()
        self.marker_hist_pub.publish(marker)

        # publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed          = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)

        # update frame ID
        self.frame_id += 1


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
