#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.duration import Duration

from ros2_uav_px4.srv import ModeSelector
from ros2_uav_interfaces.msg import ModeStatus, PoseHeading
from px4_msgs.msg import VehicleOdometry

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import sys
import math
import yaml
import argparse
import os

import rclpy
import time
from rclpy.node import Node
from rclpy.duration import Duration

from ros2_uav_px4.srv import ModeSelector
from ros2_uav_interfaces.msg import ModeStatus, PoseHeading, Waypoint, WaypointList
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Point

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import sys
import math
import yaml
import argparse
import os

class MissionExecutor(Node):
    def __init__(self, namespace='/uav0', missions_file='missions.yaml'):
        super().__init__('mission_executor')

        self.namespace = namespace.rstrip('/')  # Ensure no trailing slash

        # Load missions from YAML file
        self.missions = self.load_missions(missions_file)
        if not self.missions:
            self.get_logger().error(f"No missions found in {missions_file}. Exiting.")
            sys.exit(1)

        # Service client for setting mode
        self.set_mode_client = self.create_client(ModeSelector, f'{self.namespace}/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
            if not rclpy.ok():
                self.get_logger().error('ROS 2 shutdown while waiting for set_mode service.')
                sys.exit(1)

        # Publisher for command pose_heading
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.command_publisher = self.create_publisher(PoseHeading, f'{self.namespace}/command/pose_heading', qos_profile)

        # Publisher for waypoint list
        self.waypoint_list_publisher = self.create_publisher(
            WaypointList, f'{self.namespace}/command/waypoints', qos_profile)

        # Subscriber for mode status
        self.mode_status_sub = self.create_subscription(
            ModeStatus,
            f'{self.namespace}/modes_status/position',
            self.mode_status_callback,
            qos_profile
        )
        self.mode_status_sub_nlmpc = self.create_subscription(
            ModeStatus,
            f'{self.namespace}/modes_status/nlmpcposition',
            self.mode_status_callback,
            qos_profile
        )
        self.mode_status_sub_landspin = self.create_subscription(
            ModeStatus,
            f'{self.namespace}/modes_status/landspin',
            self.mode_status_callback,
            qos_profile
        )
        self.mode_status_sub_nlmpcwaypoints = self.create_subscription(
            ModeStatus,
            f'{self.namespace}/modes_status/nlmpcwaypoints',
            self.mode_status_callback,
            qos_profile
        )

        # Subscriber for odometry
        px4_odometry_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            f'{self.namespace}/fmu/out/vehicle_odometry',
            self.odometry_callback,
            px4_odometry_qos_profile
        )
        self.current_odometry = None

        self.completed = False

    def load_missions(self, missions_file):
        if not os.path.exists(missions_file):
            self.get_logger().error(f"Missions file {missions_file} does not exist.")
            return None
        try:
            with open(missions_file, 'r') as file:
                data = yaml.safe_load(file)
            missions = data.get('missions', [])
            self.get_logger().info(f"Loaded {len(missions)} missions from {missions_file}.")
            return missions
        except Exception as e:
            self.get_logger().error(f"Failed to load missions from {missions_file}: {e}")
            return None

    def mode_status_callback(self, msg):
        if msg.status == ModeStatus.IDLE:
            self.completed = True

    def odometry_callback(self, msg):
        self.current_odometry = msg

    def set_mode(self, mode):
        request = ModeSelector.Request()
        mode_mapping = {
            "POSITION": ModeSelector.Request.POSITION,
            "NLMPCPOSITION": ModeSelector.Request.NLMPCPOSITION,
            "LANDSPIN": ModeSelector.Request.LANDSPIN,
            "NLMPCWAYPOINTS": ModeSelector.Request.NLMPCWAYPOINTS
        }
        if mode not in mode_mapping:
            self.get_logger().error(f"Unknown mode: {mode}")
            return False

        request.mode = mode_mapping[mode]
        self.get_logger().info(f"Setting mode to {mode}...")

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Mode set to {mode}")
            return True
        else:
            self.get_logger().error(f"Failed to set mode to {mode}")
            return False

    def send_setpoint(self, setpoint):
        msg = PoseHeading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.namespace}/odom"

        # Set position
        msg.position.x = setpoint['position']['x']
        msg.position.y = setpoint['position']['y']
        msg.position.z = setpoint['position']['z']

        # Set velocity
        msg.velocity.x = setpoint['velocity']['x']
        msg.velocity.y = setpoint['velocity']['y']
        msg.velocity.z = setpoint['velocity']['z']

        # Set heading
        msg.heading = setpoint['heading']

        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published setpoint: {setpoint}")

    def get_current_position(self):
        if self.current_odometry:
            position = {
                'x': self.current_odometry.position[0],
                'y': -self.current_odometry.position[1],
                'z': -self.current_odometry.position[2]
            }
            return position
        else:
            return None

    def send_waypoint_list(self, waypoints):
        msg = WaypointList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.namespace}/odom"

        # Convert the list of waypoint dictionaries to Waypoint messages
        msg.waypoints = []
        for wp in waypoints:
            waypoint = Waypoint()
            waypoint.position = Point(
                x=wp['position']['x'],
                y=wp['position']['y'],
                z=wp['position']['z']
            )
            waypoint.heading = wp.get('heading', 0.0)
            waypoint.speed = wp.get('speed', 1.0)  # Default speed if not specified
            msg.waypoints.append(waypoint)

        self.waypoint_list_publisher.publish(msg)
        self.get_logger().info(f"Published waypoint list with {len(msg.waypoints)} waypoints.")

    def execute_mission(self, mission):
        mode = mission['mode']
        mission_name = mission.get('name', mode)
        setpoint = mission.get('setpoint', None)
        waypoints = mission.get('waypoints', None)
        pause = mission.get('pause', None)

        self.get_logger().info(f"--- Executing Mission: {mission_name} ---")

        # Step 1: Set Mode
        if not self.set_mode(mode):
            self.get_logger().error(f"Failed to set mode to {mode}. Aborting mission.")
            return False
        # Allow some time for the drone to switch modes
        time.sleep(1.0)

        # Step 2: Wait for Mode to Reach IDLE
        self.completed = False
        self.get_logger().info(f"Waiting for mode {mode} to reach IDLE state...")
        timeout = 30  # seconds
        start_time = self.get_clock().now()
        while not self.completed:
            if (self.get_clock().now() - start_time) > Duration(seconds=timeout):
                self.get_logger().error(f"Timeout waiting for mode {mode} to reach IDLE state.")
                return False
            rclpy.spin_once(self, timeout_sec=0.5)

        # Step 3: Send Setpoint or Waypoints or pause
        if setpoint:
            self.send_setpoint(setpoint)

            # Allow some time for the drone to process the setpoint
            rclpy.spin_once(self, timeout_sec=2.0)

            # Step 4: Verify Position
            self.get_logger().info("Verifying drone position...")
            timeout = 200  # seconds
            start_time = self.get_clock().now()
            last_print_time = self.get_clock().now()
            while True:
                current_position = self.get_current_position()
                if current_position:
                    distance = self.calculate_distance(setpoint['position'], current_position)
                    if distance <= 1.0:
                        self.get_logger().info("Drone reached the expected position.")
                        break

                    if (self.get_clock().now() - last_print_time) > Duration(seconds=10):
                        self.get_logger().info(f"Waiting for drone to reach position. Measured distance: {distance:.2f} meters")
                        last_print_time = self.get_clock().now()

                if (self.get_clock().now() - start_time) > Duration(seconds=timeout):
                    self.get_logger().error("Timeout waiting for drone to reach the expected position.")
                    return False
                rclpy.spin_once(self, timeout_sec=0.5)

        elif waypoints:
            self.send_waypoint_list(waypoints)
            self.get_logger().info("Waypoint list sent to the drone.")
            # Allow some time for the drone to process the setpoint
            start_time = self.get_clock().now()
            time_wait = 1.0
            while (self.get_clock().now() - start_time) < Duration(seconds=time_wait):
                rclpy.spin_once(self, timeout_sec=0.5)
            self.completed = False
            while True:
                # waiting for the drone to reach the last waypoint
                if self.completed:
                    break
                rclpy.spin_once(self, timeout_sec=0.5)
        elif pause:
            self.get_logger().info(f"Pausing for {pause} seconds...")
            time.sleep(pause)

        self.get_logger().info(f"Mission '{mission_name}' executed successfully.")
        return True

    def calculate_distance(self, expected, actual):
        dx = expected['x'] - actual['x']
        dy = expected['y'] - actual['y']
        dz = expected['z'] - actual['z']
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        self.get_logger().debug(f"Distance to target: {distance:.2f} meters.")
        return distance

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Execute drone missions from a YAML file.')
    parser.add_argument('--namespace', type=str, default='/uav0', help='Namespace of the drone.')
    parser.add_argument('--missions', type=str, default='missions.yaml', help='Path to the missions YAML file.')
    args = parser.parse_args()

    # Verify missions file path
    missions_file = args.missions
    if not os.path.isfile(missions_file):
        print(f"Error: Missions file '{missions_file}' does not exist.")
        sys.exit(1)

    executor_node = MissionExecutor(namespace=args.namespace, missions_file=missions_file)

    success_all = True

    for mission in executor_node.missions:
        success = executor_node.execute_mission(mission)
        if not success:
            executor_node.get_logger().error(f"Mission '{mission.get('name', mission['mode'])}' failed.")
            success_all = False
            # Depending on requirements, you can choose to continue with next missions or abort.
            # Here, we continue with the next mission.
        else:
            executor_node.get_logger().info(f"Mission '{mission.get('name', mission['mode'])}' completed successfully.")

    if success_all:
        executor_node.get_logger().info("All missions executed successfully.")
    else:
        executor_node.get_logger().warn("Some missions failed. Check the logs for details.")

    # Shutdown ROS 2
    executor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()