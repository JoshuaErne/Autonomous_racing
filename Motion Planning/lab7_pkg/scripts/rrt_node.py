#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""

import math
import copy

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


class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')

        # improt pure pursuit functions
        self.L = 3
        self.pure_pursuit = PurePursuit(L=self.L, segments=1024, filepath="src/lab7/waypoints.csv")
        self.utils = Utils()

        # create subscribers
        self.pose_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.pose_callback, 1)
        self.scan_sub = self.create_subscription(LaserScan,  "/scan", self.scan_callback, 1)

        # publishers
        self.waypoint_pub = self.create_publisher(Marker, "/waypoint_marker", 10)
        self.rrt_path_pub = self.create_publisher(Marker, "/rrt_path", 10)
        self.rrt_node_pub = self.create_publisher(MarkerArray, "/rrt_node_array", 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, "/occupancy_grid", 10)
        
        # class variables
        self.grid_height = int(self.L * 10)
        self.grid_width = int(60)
        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=-1, dtype=int)
        self.current_pose = None
        self.goal_pos = None

        # constants
        self.MAX_RANGE = self.L - 0.1
        self.MIN_ANGLE = np.radians(0)
        self.MAX_ANGLE = np.radians(180)
        self.ANGLE_OFFSET = np.radians(45)
        self.IS_OCCUPIED = 100
        self.IS_FREE = 0
        self.CELLS_PER_METER = 10
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1
        self.MAX_RRT_ITER = 100

        # hyper-parameters
        self.is_sim = True
        self.velocity_min = 1.5
        self.velocity_max = 5.0
        self.populate_free = True

        # rrt variables
        self.path_world = []

    def local_to_grid(self, point):
        """
        Convert coordinates from car's local frame 
        of reference to occupancy grid indices.

        Args:
            point (tuple(float, float)): point in local coordinates
        Returns:
            point (tuple(int, int)): point in occupancy grid indices
        """
        x, y = point[0], point[1]
        i = int(x * -self.CELLS_PER_METER + (self.grid_height -1))
        j = int(y * -self.CELLS_PER_METER + self.CELL_Y_OFFSET)
        return (i, j)

    def grid_to_local(self, point):
        """
        Convert coordinates from occuapcny grid indices
        to the car's local frame of reference.

        Args:
            point (tuple(int, int)): point in occupancy grid indices
        Returns:
            point (tuple(float, float)): point in point in local coordinates
        """
        i, j = point[0], point[1]
        x = (i - (self.grid_height - 1))  / -self.CELLS_PER_METER
        y = (j - self.CELL_Y_OFFSET) / -self.CELLS_PER_METER
        return (x, y)

    def pose_callback(self, pose_msg):
        """
        Callback function for subscribing to particle filter's inferred pose.
        This funcion saves the current pose of the car and obtain the goal
        waypoint from the pure pursuit module.

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        """
        # determine pose data type (sim vs. car)
        pose = pose_msg.pose.pose if self.is_sim else pose_msg.pose

        # save current car pose
        self.current_pose = copy.deepcopy(pose)

        # obtain pure pursuit waypoint 
        self.goal_pos = self.pure_pursuit.get_waypoint(pose)
               
    def populate_occupancy_grid(self, ranges, angle_increment):
        """
        Populate occupancy grid using lidar scans and save 
        the data in class member variable self.occupancy_grid.

        Args:
            ranges (list): lidar scan data 
            angle_increment (float): angle step per lidar ray
        """
        # reset empty occupacny grid (-1 = unknown)
        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=-1, dtype=int)

        # enumerate over lidar scans
        for i, dist in enumerate(ranges):
            # skip scans behind the car
            theta = (i * angle_increment) - self.ANGLE_OFFSET
            if theta < self.MIN_ANGLE or theta > self.MAX_ANGLE:
                continue

            # obtain local coordinate of scan
            dist_clipped = np.clip(dist, 0, self.MAX_RANGE)
            x = dist_clipped * np.sin(theta)
            y = dist_clipped * np.cos(theta) * -1

            # obtain grid indices from local coordinates of scan point
            i, j = self.local_to_grid((x, y))

            # set occupied space
            if dist < self.MAX_RANGE:
                self.occupancy_grid[i, j] = self.IS_OCCUPIED

            # set free space by fast voxel traverse
            if self.populate_free:
                free_cells = self.utils.traverse_grid(self.local_to_grid((0, 0)), (i, j))
                for cell in free_cells:
                    if self.occupancy_grid[cell] != self.IS_OCCUPIED:
                        self.occupancy_grid[cell] = self.IS_FREE

    def publish_occupancy_grid(self, frame_id, stamp):
        """
        Publish populated occupancy grid to ros2 topic for display.

        Args: 
            frame_id: frame id for published message
            stamp: timestamp for published message
        """
        oc = OccupancyGrid()
        oc.header.frame_id = frame_id
        oc.header.stamp = stamp
        oc.info.origin.position.y -= ((self.grid_width / 2) + 1) / 10
        oc.info.width = self.grid_height
        oc.info.height = self.grid_width
        oc.info.resolution = 0.1
        oc.data = np.fliplr(np.rot90(self.occupancy_grid, k=1)).flatten().tolist()
        self.occupancy_grid_pub.publish(oc)

    def convolve_occupancy_grid(self):
        """
        Perform convolution on occupancy grid to enlarge the obstacles
        or "occupied" spaces in order to account for the size of the car.
        """
        kernel = np.ones(shape=[3, 3])
        self.occupancy_grid = signal.convolve2d(
            self.occupancy_grid.astype('int'),
            kernel.astype('int'),
            boundary='symm',
            mode='same'
        )
        self.occupancy_grid = np.clip(self.occupancy_grid, -1, 100)

    def drive_to_target(self, point):
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

        # determine velocity using logistic function
        logit = np.clip(
            (2.0 / (1.0 + np.exp(-5 * (np.abs(np.degrees(angle)) / 20.0)))) - 1.0,
            0.0,
            1.0
        )
        velocity = self.velocity_max - (logit * (self.velocity_max - self.velocity_min))

        # publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed          = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, update occupancy grid and perform RRT

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        """
        # make sure we obtain initial pose and goal point
        if (self.current_pose is None) or (self.goal_pos is None):
            return

        # populate occupancy grid
        self.populate_occupancy_grid(scan_msg.ranges, scan_msg.angle_increment)
        self.convolve_occupancy_grid()
        self.publish_occupancy_grid(scan_msg.header.frame_id, scan_msg.header.stamp)

        # get path planed in occupancy grid space
        path_grid = self.rrt()

        # convert path from grid to local coordinates
        path_local = [self.grid_to_local(point) for point in path_grid]

        # return if RRT does not return a valid path
        if len(path_local) < 2:
            return

        # navigate to first node in tree
        self.drive_to_target(path_local[1])

        # rrt visualization
        self.utils.draw_marker_array(
            scan_msg.header.frame_id,
            scan_msg.header.stamp,
            path_local,
            self.rrt_node_pub
        )

        self.utils.draw_lines(
            scan_msg.header.frame_id,
            scan_msg.header.stamp,
            path_local,
            self.rrt_path_pub
        )

    def rrt(self):
        """
        Perform RRT to find the optimal path to reach goal point.

        Returns:
            path (list): list of nodes forming the path to goal point
        """
        # convert position to occupancy grid indices
        current_pos = self.local_to_grid((0, 0))
        goal_pos = self.local_to_grid(self.goal_pos)

        # resample a close point if our goal point is occupied 
        if self.occupancy_grid[goal_pos] == self.IS_OCCUPIED:
            i, j = self.sample()
            while np.linalg.norm(np.array([i,j]) - np.array(goal_pos)) > 5:
                i, j = self.sample()
            goal_pos = (i, j)
        
        # initialize start and goal trees
        T_start = [Vertex(current_pos)]
        T_goal = [Vertex(goal_pos)]
        
        # start rrt algorithm
        for iter in range(self.MAX_RRT_ITER):
            # sample from free space
            pos_sampled = self.sample()

            # attempt to expand tree using sampled point
            T_start, success_start = self.expand_tree(T_start, pos_sampled, check_closer=True)
            T_goal, success_goal = self.expand_tree(T_goal, pos_sampled)

            # if sampled point can reach both T_start and T_goal
            # get path from start to goal and return
            if success_start and success_goal:
                path = self.find_path(T_start, T_goal, pruning=True)
                return path
        return []

    def sample(self):
        """
        Randomly sample the free space in occupancy grid, and returns its index.
        If free space has already been populated then just check if sampled
        cell is free, else do fast voxel traversal for each sampling.

        Returns:
            point (tuple(int, int)): index of free cell in occupancy grid
        """
        if self.populate_free:
            i, j = np.random.randint(self.grid_height), np.random.randint(self.grid_width)
            while self.occupancy_grid[i, j] != self.IS_FREE:
                i, j = np.random.randint(self.grid_height), np.random.randint(self.grid_width)
        else:
            free = False
            while not free:
                i, j = np.random.randint(self.grid_height), np.random.randint(self.grid_width)
                free = True
                for cell in self.utils.traverse_grid(self.local_to_grid((0, 0)), (i, j)):
                    if self.occupancy_grid[cell] == self.IS_OCCUPIED:
                        free = False
                        break
        return (i, j)

    def expand_tree(self, tree, sampled_point, check_closer=False):
        """
        Attempts to expand tree using the sampled point by
        checking if it causes collision and if the new node
        brings the car closer to the goal.

        Args:
            tree (list): current RRT tree
            sampled_point (tuple): cell sampled in occupancy grid free space 
            check_closer (bool): check if sampled point brings car closer
        Returns:
            tree (list): expanded RRT tree
            success (bool): whether tree was successfully expanded
        """
        # get closest node to sampled point in tree
        idx_nearest = self.nearest(tree, sampled_point)
        pos_nearest = tree[idx_nearest].pos

        # check if nearest node -> sampled node causes collision
        collision = self.check_collision(sampled_point, pos_nearest)

        # check if sampeld point bring car closer to goal compared to the nearest node
        is_closer = self.is_closer(sampled_point, pos_nearest) if check_closer else True

        # if p_free -> p_nearest causes no collision
        # then add p_free as child of p_nearest in T_start
        if is_closer and (not collision):
            tree.append(Vertex(sampled_point, idx_nearest))

        return tree, (is_closer and (not collision))

    def nearest(self, tree, sampled_point):
        """
        Return the nearest node on the tree to the sampled point.

        Args:
            tree (list) the current RRT tree
            sampled_cell (tuple): point sampled in occupancy grid free space 
        Returns:
            nearest_indx (int): index of neareset node on the tree
        """
        nearest_indx = -1
        nearest_dist = np.Inf
        for idx, node in enumerate(tree):
            dist = np.linalg.norm(np.array(sampled_point) - np.array(node.pos))
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_indx = idx
        return nearest_indx

    def is_closer(self, sampled_point, nearest_point):
        """
        Checks if the new sampled node brings the car closer
        to the goal point then the nearest existing node on the tree
        
        Args:
            sampled_point (tuple): position of sampled point in occupancy grid
            nearest_point (tuple): positoin of nearest point in occupancy grid
        Returns:
            is_closer (bool): if sampled point brings the car closer to goal point
        """
        a = self.grid_to_local(sampled_point)
        b = self.grid_to_local(nearest_point)
        return np.linalg.norm(a - self.goal_pos[:2]) < np.linalg.norm(b - self.goal_pos[:2])

    def check_collision(self, cell_a, cell_b):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for cell in self.utils.traverse_grid(cell_a, cell_b):
            if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                continue
            if self.occupancy_grid[cell] == self.IS_OCCUPIED:
                return True
        return False

    def find_path(self, T_start, T_goal, pruning=True):
        """
        Returns a list of nodes as the path connecting start to goal.

        Args:
            T_start (list): RRT tree with starting position as root
            T_goal  (list): RRT tree with goal position as root
            pruning (bool): whether to perform path pruning
        Returns:
            path (list): valid path as a list of nodes
        """
        # traverse up T_start to obtain path to sampled point
        node = T_start[-1]
        path_start = [node.pos]
        while node.parent is not None:
            node = T_start[node.parent]
            path_start.append(node.pos)

        # traverse up T_goal to obtain path to sampled point
        node = T_goal[-1]
        path_goal = [node.pos]
        while node.parent is not None:
            node = T_goal[node.parent]
            path_goal.append(node.pos)

        # return path
        path = np.array(path_start[::-1] + path_goal[1:])

        # pruning if enabled
        if pruning:
            sub_paths = []
            for i in range(len(path) - 2):
                sub_path = path
                for j in range(i + 2, len(path)):
                    if not self.check_collision(path[i], path[j]):
                        sub_path = np.vstack((path[:i+1], path[j:]))
                sub_paths.append(sub_path)

            costs = np.array([np.linalg.norm(p[1:] - p[:-1]).sum() for p in sub_paths])
            path = sub_paths[np.argmin(costs)]
        return path


class Utils:
    def __init__(self):
        pass

    def draw_marker(self, frame_id, stamp, position, publisher, id=0):
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
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
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
    
    def traverse_grid(self, start, end):
        """
        Bresenham's line algorithm for fast voxel traversal.

        CREDIT TO: Rogue Basin
        CODE TAKEN FROM: http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        return points


class PurePursuit:
    def __init__(self, L=1.7, segments=1024, filepath="src/lab7/waypoints.csv"):
        self.L = L
        self.waypoints = self.interpolate_waypoints(
            file_path=filepath,
            segments=segments
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

    def get_waypoint(self, pose):
        # get current position of car
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        waypoints = self.transform_waypoints(self.waypoints, position, pose)

        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints, axis=1)

        # get indices of waypoints that are within L, sorted by descending distance
        indices_L = np.argsort(np.where(distances < self.L, distances, -1))[::-1]

        # set goal point to be the farthest valid waypoint within distance L
        for i in indices_L:
            # check waypoint is in front of car
            x = waypoints[i][0]
            if x > 0:
                return waypoints[i]
        return None


def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
