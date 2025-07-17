import rclpy
from rclpy.node import Node
from btr2_msgs.srv import ResetOdometry  # あなたのサービスのパスに合わせて調整

class ResetOdometryClient(Node):

    def __init__(self):
        super().__init__('reset_odometry_client')
        self.cli = self.create_client(ResetOdometry, '/reset_odometry')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/reset_odometry service not available, waiting...')

    def send_request(self, x, y, phi):
        request = ResetOdometry.Request()
        request.x = x
        request.y = y
        request.phi = phi
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('ResetOdometry service call successful')
        else:
            self.get_logger().error('Service call failed %r' % (future.exception(),))


def main(args=None):
    rclpy.init(args=args)
    client = ResetOdometryClient()
    client.send_request(0.0, 0.0, 0.0)  # 必要な値に変更
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

