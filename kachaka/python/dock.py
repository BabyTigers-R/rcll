#!/usr/bin/python3
import rclpy
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand
from rclpy.action import ActionClient
from rclpy.node import Node

class DockCommand(Node):
    def __init__(self) -> None:
        super().__init__("dockcommand")
        self._action_client = ActionClient(
            self, ExecKachakaCommand, "/kachaka/kachaka_command/execute"
        )
        self._action_client.wait_for_server()

    def dock_shelf(self):
        command = KachakaCommand()
        command.command_type = KachakaCommand.DOCK_SHELF_COMMAND
        # command.move_shelf_command_target_shelf_id = "S01"
        # command.move_shelf_command_destination_location_id = "S01_home"
        # command.move_shelf_command_undock_on_destination = True
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    dock_command = DockCommand()
    future = dock_command.dock_shelf()
    rclpy.spin_until_future_complete(dock_command, future)

    dock_command.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
