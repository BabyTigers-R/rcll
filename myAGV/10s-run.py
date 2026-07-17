import rclpy
import geometry_msgs.msg 
import time
rclpy.init(args=None)

node = rclpy.create_node("arm_controller")

stamped = node.declare_parameter('stamped', False).value
if stamped:
    TwistMsg = geometry_msgs.msg.TwistStamped
else:
    TwistMsg = geometry_msgs.msg.Twist
cmd_vel_pub = node.create_publisher(
    TwistMsg,
    "/cmd_vel",
    10
)


def send_cmd_vel(vx, vy, wz):

        msg = TwistMsg()

        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz

        self.cmd_vel_pub.publish(msg)

        rclpy.spin_once(self.node, timeout_sec=0)


time.sleep(3)
print(TwistMsg())
send_cmd_vel(1,0,0)
time(3)

send_cmd_vel(0,0,0)
