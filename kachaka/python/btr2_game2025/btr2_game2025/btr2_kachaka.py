import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import time

class btr_go(Node):
    def __init__(self):
        super().__init__('btr2025')
        self.twist_pub = self.create_publisher(Twist, '/kachaka/manual_control/cmd_vel', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        velocity = Twist()
        # velocity.header.frame_id = 'btr2025'
        # velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.linear.x = 0.2
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = 0.0
        self.twist_pub.publish(velocity)
        print("publish:", velocity)

def main(args=None):
    rclpy.init(args=args)
    btr2025 = btr_go()
    rclpy.spin(btr2025)
    rclpy.shutdonw()


if __name__ == '__main__':
    main()
