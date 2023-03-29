#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from scipy.spatial.transform import Rotation as R
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose
from std_msgs.msg import String


class WaypointLoggerNode(Node):
    def __init__(self):
        super().__init__('waypoint_logger_node')
        self.file = open(strftime('wp-%Y-%m-%d-%H-%M-%S', gmtime())+'.csv', 'w')
        self.sub_lidar = self.create_subscription(Pose, '/initialpose', self.pose_callback, 10)

    def pose_callback(self, pose_msg):
        quaternion = np.array([
            pose_msg.pose.pose.orientation.x, 
            pose_msg.pose.pose.orientation.y, 
            pose_msg.pose.pose.orientation.z, 
            pose_msg.pose.pose.orientation.w
        ])
        euler = R.from_quat(quaternion).as_euler('xyz', degrees=True)

        self.file.write('%f, %f, %f\n' % (
            pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
            euler[2]
        ))
        self.get_logger().info("Waypoint Logged.")


def main(args=None):
    rclpy.init(args=args)
    print("Waypoint Logger Initialized")
    waypoint_logger_node = WaypointLoggerNode()
    rclpy.spin(waypoint_logger_node)

    waypoint_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
