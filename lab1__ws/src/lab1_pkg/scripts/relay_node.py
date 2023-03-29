#!/usr/bin/env python3
...

import rclpy
#import rospy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

speed_2 = 0
steering_angle_2 = 0

class relay(Node):
    
    
    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(AckermannDriveStamped,'drive',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, ack_msg_1):
        global speed_2, steering_angle_2
        speed_2 = ack_msg_1.drive.speed
        steering_angle_2 = ack_msg_1.drive.steering_angle
        #self.get_logger().info('Sub Speed: "%d"' % speed_2)
        #self.get_logger().info('sub steering_angle: "%d"' % steering_angle_2)

    #def speeds(self,ack_msg_1):
        # speed = ack_msg_1.drive.speed
        # steering_angle = ack_msg_1.drive.steering_angle
        # return speed_2,steering_angle


    def timer_callback(self):
        ack_msg_2 = AckermannDriveStamped()
        speed_3 =  speed_2*3
        steering_angle_3 =  steering_angle_2*3
        self.publisher_.publish(ack_msg_2)
        #self.get_logger().info('Publishing Speed: "%d"' % speed_3)
        #self.get_logger().info('Publishing steering_angle: "%d"' % steering_angle_3)

def main(args=None):
    rclpy.init(args=args)
    relay_ = relay()
    rclpy.spin(relay_)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
