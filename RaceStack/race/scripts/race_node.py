#!/usr/bin/env python3
import math
import copy
import time

import numpy as np
from scipy import signal
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class Vertex(object):
    def __init__(self, pos=None, parent=None):
        self.pos = pos
        self.parent = parent


class Race(Node):
    def __init__(self):
        super().__init__('race')

        # improt pure pursuit functions
        self.L = 2.7
        self.pure_pursuit = PurePursuit()
        self.utils = Utils()
        self.is_sim=False
        self.frame_id = 0
        self.time_in_overtake = 0
        self.time_in_return = 0
        self.offsets = np.array([0.7, 0.75, 0.8, 0.85, 0.9])
        self.overtake_sign = -1.0

        # create subscribers
        if self.is_sim:
            self.pose_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.pose_callback, 10)
        else:
            self.pose_sub = self.create_subscription(PoseStamped, "/pf/viz/inferred_pose", self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan,  "/scan", self.scan_callback, 1)

        # publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)
        self.marker_hist_pub = self.create_publisher(Marker, "/visualization_marker_history", 10)

        # class variables
        self.current_pose = None
        self.prev_idx = None
        self.prev_goal = None

    def pose_callback(self, pose_msg):
        """
        Callback function for subscribing to particle filter's inferred pose.
        This funcion saves the current pose of the car and obtain the goal
        waypoint from the pure pursuit module.

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        """
        # determine pose data type (sim vs. car)
        pose = pose_msg.pose
        if self.is_sim:
            pose = pose_msg.pose.pose

        # save current car pose
        self.current_pose = copy.deepcopy(pose)
 
    def drive_to_target(self, point, velocity=None):
        """
        Determine angle and velocity for car to navigate to the 
        target point and publish the commands to the drive topic. 
        We use a logistic function to determine velocity.

        Args:
            point (tuple(int, int)): target point for navigation
        """
        # calculate curvature/steering angle
        L = np.linalg.norm(point)
        y = point[1]
        angle = (2 * y) / (L**2)
        angle = np.clip(angle, -0.5, 0.5)

        if velocity is None or velocity == -1:
            velocity = 4.0
            if (np.abs(angle) < np.radians(10)):
                velocity = 6.0
            elif (np.abs(angle) < np.radians(20)):
                velocity = 5.0

        # publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed          = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)

    def check_collision(self, goal, ranges, L):
        if goal is None:
            return True
        heading_angle = np.degrees(np.arctan2(goal[1], goal[0]))
        index = int(heading_angle + 135) * 4
        votes = np.sum(np.array(ranges[index-(4*4):index+(4*4)]) < L)
        return votes > 0
            
    def scan_callback(self, scan_msg):
        """
        LaserScan callback

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        """
        
        # make sure we obtain initial pose and goal point
        if (self.current_pose is None):
            return
        
        # obtain pure pursuit waypoint 
        goal_pos, self.prev_idx, goal_solo, velocity = self.pure_pursuit.get_waypoint(
            self.current_pose, self.prev_idx, scan_msg.ranges, self.L
        )

        # return if there are no valid waypoints
        if goal_pos is None and goal_solo is None:
            if self.prev_goal is not None:
                self.drive_to_target(self.prev_goal)
            return

        # if original goal is occupied
        if goal_pos is None and goal_solo is not None:
            overtake = False
            
            overtake_right = [
                self.check_collision((goal_solo[0], goal_solo[1] + offset), scan_msg.ranges, self.L)
                for offset in -1.0 * self.offsets
            ]
            overtake_left = [
                self.check_collision((goal_solo[0], goal_solo[1] + offset), scan_msg.ranges, self.L)
                for offset in self.offsets
            ]
            if np.sum(overtake_right) > np.sum(overtake_left):
                self.overtake_sign = 1.0
                
            for offset in (self.overtake_sign * self.offsets):
                goal_shifted = copy.deepcopy(goal_solo)
                goal_shifted[1] += offset
                if not self.check_collision(goal_shifted, scan_msg.ranges, self.L):
                    overtake = True
                    self.time_in_overtake += 1
                    self.time_in_return = 0
                    print("overtake!")
                    self.L = 2.3
                    goal_pos = goal_shifted
                    break
            if not overtake:
                return
        else:
            self.L = 2.7
            if self.time_in_overtake > 0:
                self.time_in_overtake = 0
                self.time_in_return = 1
                decay = np.clip(31 - (np.power(1.07, self.time_in_return)), 0, 30)
                goal_pos[1] += self.overtake_sign * 0.02 * (decay)
            elif self.time_in_return > 0:
                self.time_in_return += 1
                decay = np.clip(31 - (np.power(1.07, self.time_in_return)), 0, 30)
                goal_pos[1] += self.overtake_sign * 0.02 * (decay)
                if self.time_in_return > 49:
                    self.time_in_return = 0

        # navigate to target waypoint
        self.drive_to_target(goal_pos, velocity)
        self.prev_goal = goal_pos

        # waypoint visualization
        self.utils.draw_marker(
            scan_msg.header.frame_id,
            scan_msg.header.stamp,
            goal_pos,
            self.marker_pub,
            id = 0
        )
        
        self.utils.draw_marker(
            scan_msg.header.frame_id,
            scan_msg.header.stamp,
            goal_pos,
            self.marker_hist_pub,
            id = self.frame_id,
            alpha = 0.5
        )

        self.frame_id += 1


class Utils:
    def __init__(self):
        pass

    def draw_marker(self, frame_id, stamp, position, publisher, id=0, alpha=1.0):
        """
        Publish a single marker with given ID for visulization on rviz2.

        Args:
            frame_id: frame of the published message
            stamp: timestamp of the published message
            position: position of the marker
            publisher: ros2 marker publisher 
            id: id of the marker
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = alpha
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
        marker.lifetime = Duration(seconds=10).to_msg()
        publisher.publish(marker)

    def draw_marker_array(self, frame_id, stamp, positions, publisher):
        """
        Publish an array of markers for visulization on rviz2.

        Args:
            frame_id: frame of the published message
            stamp: timestamp of the published message
            positions: list of marker positions
            publisher: ros2 marker array publisher 
        """
        marker_array = MarkerArray()
        for i, position in enumerate(positions): 
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.0
            marker.lifetime = Duration(seconds=0.05).to_msg()
            marker_array.markers.append(marker)
        publisher.publish(marker_array)

    def draw_lines(self, frame_id, stamp, path, publisher):
        """
        Publish an array of lines for visulization on rviz2.

        Args:
            frame_id: frame of the published message
            stamp: timestamp of the published message
            path: list of vertex positions for line
            publisher: ros2 marker publisher 
        """
        points = []
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i+1]
            point = Point()
            point.x = a[0]
            point.y = a[1]
            points.append(copy.deepcopy(point))
            point.x = b[0]
            point.y = b[1]
            points.append(copy.deepcopy(point))

        line_list = Marker()
        line_list.header.frame_id = frame_id
        line_list.header.stamp = stamp
        line_list.id = 0
        line_list.type = line_list.LINE_LIST
        line_list.action = line_list.ADD
        line_list.scale.x = 0.01
        line_list.color.a = 1.0
        line_list.color.r = 0.0
        line_list.color.g = 0.0
        line_list.color.b = 1.0
        line_list.points = points
        publisher.publish(line_list)

class PurePursuit:
    def __init__(self, dir="src/race/waypoints/"):
        self.waypoints, self.velocities = self.process_waypoints(
            file_path = dir + "center.csv"
        )

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
    
    def check_collision(self, goal, ranges, L):
        heading_angle = np.degrees(np.arctan2(goal[1], goal[0]))
        index = int(heading_angle + 135) * 4
        votes = np.sum(
            np.array(ranges[index-(4*4):index+(4*4)]) < L
        )
        return votes > 0

    def process_waypoints(self, file_path):
        # Read waypoints from csv
        points = np.genfromtxt(file_path, delimiter=",")
        
        # Append start segment to waypoints
        segment_length = int(len(points) * 0.10)
        points = np.vstack((points, points[:segment_length]))
        
        # seperate waypoints and velocity profile
        if points.shape[1] == 4:
            waypoints, velocities = points[:,:3], points[:,3]
        else:
            waypoints, velocities = points, np.full((points.shape[0], 1), -1)
        return waypoints, velocities
    
    def get_waypoint(self, pose, prev_idx, ranges, L):
        # get current position of car
        position = np.array([pose.position.x, pose.position.y, 0])

        # get distance from car to all waypoints
        distances = np.linalg.norm(self.waypoints - position, axis=1)

        # get indices of waypoints that are within L, sorted by descending distance
        indices_L = np.argsort(np.where(distances < L, distances, -1))[::-1]

        goal = None
        goal_solo = None
        goal_found = False
        velocity = None
        for i in indices_L:
            goal_candidate = self.transform_waypoints([self.waypoints[i]], position, pose)[0]
            # check waypoint is in front of car
            if goal_candidate[0] <= 0:
                continue
            # check waypoint is within L
            if distances[i] > L:
                continue
            # check waypoint amke forward progress
            if prev_idx is None:
                goal_found = True
            elif prev_idx is not None:
                # check candidate waypoint has greater index than previous waypoint
                # AND their difference is less than a threshold 
                if (i >= prev_idx) and (i - prev_idx < 500):
                    goal_found = True
                elif i - prev_idx < -800:
                    goal_found = True
            if goal_found:
                # check if waypoint collides with opponent
                if self.check_collision(goal_candidate, ranges, L):
                    goal_solo = goal_candidate
                    velocity = self.velocities[i]
                else:
                    goal, prev_idx, velocity = goal_candidate, i, self.velocities[i]
                break
        return goal, prev_idx, goal_solo, velocity


def main(args=None):
    rclpy.init(args=args)
    print("Race Mode Initialized")
    race_node = Race()
    rclpy.spin(race_node)

    race_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
