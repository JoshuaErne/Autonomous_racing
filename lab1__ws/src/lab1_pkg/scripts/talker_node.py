#!/usr/bin/env python3
...

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

#v = float(input('speed = v = '))
#d = float(input('steering angle = d = '))

class talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.declare_parameters(namespace='',parameters=[('v', 10),('d', 10),])
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        v = self.get_parameter('v')
        d = self.get_parameter('d')
        #self.get_logger().info('v: "%f"' % v.value)
        #self.get_logger().info('d: "%f"' % d.value)
        ack_msg_1 = AckermannDriveStamped()
        #ack_msg.header.stamp = rospy.Time.now()
        #ack_msg.header.frame_id = 'your_frame_here'
        ack_msg_1.drive.steering_angle =  float(d.value)
        ack_msg_1.drive.speed =  float(v.value)
        self.publisher_.publish(ack_msg_1)
        #self.get_logger().info('Publishing Speed: "%f"' % ack_msg_1.drive.speed)
        #self.get_logger().info('Publishing steering_angle: "%f"' % ack_msg_1.drive.steering_angle)

def main(args=None):
    rclpy.init(args=args)
    talker_ = talker()

    rclpy.spin(talker_)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

