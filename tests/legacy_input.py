import sys
import rclpy
from rclpy.node import Node
from ros2_uav_interfaces.srv import UserRequest
from arrc_interfaces.srv import HighLevelCommand
from arrc_interfaces.msg import UavPose


class LegacyInputNode(Node):
    def __init__(self):
        super().__init__('legacy_input_node')
        # Client for HighLevelCommand service
        self.high_level_command_client = self.create_client(HighLevelCommand, '/uav0/command/highLevelCommand')
        # Publisher for UAV Pose
        self.uav_pose_publisher = self.create_publisher(UavPose, '/uav0/command/setPose', 10)
        # Publisher for UAV Hit Pose
        self.uav_hit_publisher = self.create_publisher(UavPose, '/uav0/command/setHit', 10)

    def send_pose(self, frame, x, y, z):
        msg = UavPose()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.x = x
        msg.pose.y = y
        msg.pose.z = z
        msg.pose.yaw = 0.0  # Assuming you want to set yaw to 0 for simplicity

        self.uav_pose_publisher.publish(msg)
        self.get_logger().info(f'Pose set to {x}, {y}, {z} in frame {frame}')

    def send_hit(self, frame, x, y, z):
        msg = UavPose()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.pose.x = x
        msg.pose.y = y
        msg.pose.z = z
        msg.pose.yaw = 0.0  # Assuming you want to set yaw to 0 for simplicity

        self.uav_hit_publisher.publish(msg)
        self.get_logger().info(f'Hit pose set to {x}, {y}, {z} in frame {frame}')

    def send_high_level_command(self, command):
        if not self.high_level_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available.')
            return

        request = HighLevelCommand.Request()
        if command == 'takeoff':
            request.cmd = HighLevelCommand.Request.TAKEOFF
        elif command == 'land':
            request.cmd = HighLevelCommand.Request.LAND
        else:
            self.get_logger().error(f"Unknown high-level command: {command}")
            return

        future = self.high_level_command_client.call_async(request)
        future.add_done_callback(self.high_level_command_callback)

    def high_level_command_callback(self, future):
        try:
            future.result()
            self.get_logger().info('High-level command executed successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to execute high-level command: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LegacyInputNode()

    if len(sys.argv) < 2:
        print('Usage: legacy_input.py <command> [args]')
        sys.exit(1)

    command = sys.argv[1]

    if command == 'take_off':
        node.send_high_level_command('takeoff')
    elif command == 'land':
        node.send_high_level_command('land')
    elif command == 'set_pose' and len(sys.argv) == 6:
        frame = sys.argv[2]
        x = float(sys.argv[3])
        y = float(sys.argv[4])
        z = float(sys.argv[5])
        node.send_pose(frame, x, y, z)
    elif command == 'set_hit' and len(sys.argv) == 6:
        frame = sys.argv[2]
        x = float(sys.argv[3])
        y = float(sys.argv[4])
        z = float(sys.argv[5])
        node.send_hit(frame, x, y, z)
    else:
        print('Invalid arguments or command.')
        sys.exit(1)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
