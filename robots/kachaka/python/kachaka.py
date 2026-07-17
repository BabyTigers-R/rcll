#!/bin/bash
import rclpy
from rclpy.node import Node
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand
from rclpy.action import ActionClient

class WaypointPub(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")
        self._action_client = ActionClient(self, ExecKachakaCommand,
                                           "/kachaka/kachaka_command/execute")
        self._action_client.wait_for_server()

    def sent_request(self, pos_x, pos_y, yaw):
        command = KachakaCommand()
        command.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        command.move_to_pose_command_x = pos_x
        command.move_to_pose_command_y = pos_y
        command.move_to_pose_command_yaw = yaw
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command
        future = self._action_client.send_goal_async(goal_msg)
        return future

    def return_home(self):
        command = KachakaCommand()
        command.command_type = KachakaCommand.RETURN_HOME_COMMAND
        command.return_home_command_silent = True
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command
        future = self._action_client.send_goal_async(goal_msg)
        return future

    def kacha_speaker(self, text):
        command = KachakaCommand()
        command.command_type = KachakaCommand.SPEAK_COMMAND
        command.speak_command_text = text
        sp_msg = ExecKachakaCommand.Goal()
        sp_msg.kachaka_command = command
        self._action_client.send_goal_async(sp_msg)


def main(args=None):
    rclpy.init(args=args)
    waypoint_pub = WaypointPub()
    waypoint_pub.kacha_speaker("アールティーブースへようこそ")
    while True:
        waypoint_pub.kacha_speaker("もくひょうちてんをせっていしてください")
        try:
            x, y, yaw = map(float, input("目標地点を入力してください x, y, yaw =").split())
        except:
            print("入力がまちがっています")
            continue

        waypoint_pub.kacha_speaker("もくひょうちてんをせっていしました")
        future = waypoint_pub.sent_request(x, y, yaw)
        rclpy.spin_until_future_complete(waypoint_pub, future)
        waypoint_pub.kacha_speaker("もくひょうちてんにとうちゃくしました")

        if input("終了しますか？ [y/n] = ") == "y":
            break

    waypoint_pub.kacha_speaker("ホームにもどります")
    waypoint_pub.return_home()

    rclpy.spin_until_future_complete(waypoint_pub, future)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
