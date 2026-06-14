import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import sys
import threading
import termios
import tty


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')

        # publisher: mobile
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # service client: arm
        self.arm_stop_client = self.create_client(Trigger, '/arm/stop')

        self.stopped = False

        # timer for continuous zero velocity publish
        self.timer = self.create_timer(0.1, self.publish_zero_velocity)

        # keyboard thread
        self.thread = threading.Thread(target=self.keyboard_listener)
        self.thread.daemon = True
        self.thread.start()

        self.get_logger().info("Emergency stop node started (press z or x)")

    def publish_zero_velocity(self):
        if self.stopped:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)

    def trigger_emergency_stop(self):
        if self.stopped:
            return

        self.stopped = True
        self.get_logger().warn("!!! EMERGENCY STOP ACTIVATED !!!")

        # stop mobile immediately
        self.publish_zero_velocity()

        # stop arm
        if self.arm_stop_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.arm_stop_client.call_async(req)
        else:
            self.get_logger().error("Arm stop service not available!")

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch in ['z', 'x']:
                    self.trigger_emergency_stop()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
