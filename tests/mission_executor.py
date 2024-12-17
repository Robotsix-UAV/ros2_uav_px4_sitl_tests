#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.duration import Duration

from ros2_uav_interfaces.srv import UserRequest
from ros2_uav_interfaces.msg import PoseHeading, Waypoint, WaypointList
from std_msgs.msg import String
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Point

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import sys
import math
import yaml
import argparse
import os

# Import parameter services and messages
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class MissionExecutor(Node):
    def __init__(self, namespace='/uav0', missions_file='missions.yaml'):
        super().__init__('mission_executor')

        self.namespace = namespace.rstrip('/')  # Ensure no trailing slash

        # Load missions from YAML file
        self.missions = self.load_missions(missions_file)
        if not self.missions:
            self.get_logger().error(f"No missions found in {missions_file}. Exiting.")
            sys.exit(1)

        # Service client for setting user action
        self.set_action_client = self.create_client(UserRequest, f'{self.namespace}/user_request')
        while not self.set_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for user_request service...')
            if not rclpy.ok():
                self.get_logger().error('ROS 2 shutdown while waiting for user_request service.')
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

        # Subscriber to current fsm state
        self.fsm_state_sub = self.create_subscription(
            String,
            f'{self.namespace}/fsm_state',
            self.fsm_state_callback,
            1
        )
        self.current_fsm_state = None

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

    def odometry_callback(self, msg):
        self.current_odometry = msg

    def fsm_state_callback(self, msg):
        self.current_fsm_state = msg.data

    def set_action(self, action, pipeline_name=''):
        request = UserRequest.Request()
        action_mapping = {
            "TAKE_OFF": 0,
            "LAND": 1,
            "LAND_HOME": 2,
            "PIPELINE": 3
        }
        if action not in action_mapping:
            self.get_logger().error(f"Unknown action: {action}")
            return False

        request.action = action_mapping[action]
        request.pipeline_name = pipeline_name
        self.get_logger().info(f"Setting action to {action} with pipeline_name='{pipeline_name}'...")

        future = self.set_action_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        alright = future.result() is not None

        if alright:
            self.get_logger().info(f"Action '{action}' set successfully.")
            if action == "TAKE_OFF":
                if not self.check_fsm_state(["ControlPipeline"], timeout=30):
                    return False
            if action == "PIPELINE":
                if not self.check_fsm_state(["ControlPipeline"], timeout=10):
                    return False
            return True
        else:
            self.get_logger().error(f"Failed to set action to {action}")
            return False

    def check_fsm_state(self, expected_states, timeout=10):
        self.get_logger().info(f"Waiting for FSM state to be '{expected_states}'...")
        start_time = self.get_clock().now()
        while True:
            if self.current_fsm_state in expected_states:
                self.get_logger().info(f"FSM state is '{self.current_fsm_state}'.")
                return True
            if (self.get_clock().now() - start_time) > Duration(seconds=timeout):
                self.get_logger().error(f"Timeout waiting for FSM state '{expected_states}'.")
                return False
            rclpy.spin_once(self, timeout_sec=0.5)

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

    def set_node_parameters(self, node_name, params_dict):
        client = self.create_client(SetParameters, f'{self.namespace}/{node_name}/set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for parameter service of node {node_name}...')
            if not rclpy.ok():
                self.get_logger().error('ROS 2 shutdown while waiting for parameter service.')
                return False
        request = SetParameters.Request()
        parameters = []
        for param_name, param_value in params_dict.items():
            self.get_logger().info(f"Setting parameter '{param_name}' to {param_value} on node {node_name}")
            parameter = Parameter()
            parameter.name = param_name
            parameter.value = ParameterValue()
            # Set the type and value appropriately
            if isinstance(param_value, int):
                parameter.value.type = ParameterType.PARAMETER_INTEGER
                parameter.value.integer_value = param_value
            elif isinstance(param_value, float):
                parameter.value.type = ParameterType.PARAMETER_DOUBLE
                parameter.value.double_value = param_value
            elif isinstance(param_value, bool):
                parameter.value.type = ParameterType.PARAMETER_BOOL
                parameter.value.bool_value = param_value
            elif isinstance(param_value, str):
                parameter.value.type = ParameterType.PARAMETER_STRING
                parameter.value.string_value = param_value
            else:
                self.get_logger().error(f"Unsupported parameter type for parameter {param_name}")
                continue
            parameters.append(parameter)
        request.parameters = parameters
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Parameters set successfully on node {node_name}")
            return True
        else:
            self.get_logger().error(f"Failed to set parameters on node {node_name}")
            return False

    def execute_mission(self, mission):
        action = mission['action']
        mission_name = mission.get('name', action)
        setpoint = mission.get('setpoint', None)
        waypoints = mission.get('waypoints', None)
        pause = mission.get('pause', None)
        pipeline_name = mission.get('pipeline_name', '')

        self.get_logger().info(f"--- Executing Mission: {mission_name} ---")

        # Handle MODIFY_PARAMS action
        if action == "MODIFY_PARAMS":
            parameters = mission.get('parameters', None)
            if parameters:
                node_name = parameters.get('node_name')
                params = parameters.get('params')
                if node_name and params:
                    if not self.set_node_parameters(node_name, params):
                        self.get_logger().error(f"Failed to set parameters on node {node_name}. Aborting mission.")
                        return False
                else:
                    self.get_logger().error(f"Parameters 'node_name' and 'params' must be specified in mission '{mission_name}'.")
                    return False
            else:
                self.get_logger().error(f"No 'parameters' field specified in mission '{mission_name}'.")
                return False
            self.get_logger().info(f"Mission '{mission_name}' executed successfully.")
            return True

        # Existing action handling
        if not self.set_action(action, pipeline_name):
            self.get_logger().error(f"Failed to set action to {action}. Aborting mission.")
            return False
        # Step 2: Check takeoff send Setpoint or Waypoints or pause
        if setpoint:
            self.send_setpoint(setpoint)
            # Verify Position
            self.get_logger().info("Verifying drone position...")
            timeout = 200  # seconds
            start_time = self.get_clock().now()
            last_print_time = self.get_clock().now()
            while True:
                current_position = self.get_current_position()
                if current_position:
                    distance = self.calculate_distance(setpoint['position'], current_position)
                    if distance <= 3.0:
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
            time.sleep(0.5)
            self.send_waypoint_list(waypoints)
            self.get_logger().info("Waypoint list sent to the drone.")
            # Check the distance to the last waypoint
            last_waypoint = waypoints[-1]
            timeout = 200
            start_time = self.get_clock().now()
            last_print_time = self.get_clock().now()
            while True:
                current_position = self.get_current_position()
                if current_position:
                    distance = self.calculate_distance(last_waypoint['position'], current_position)
                    if distance <= 2.0:
                        self.get_logger().info("Drone reached the last waypoint.")
                        break

                    if (self.get_clock().now() - last_print_time) > Duration(seconds=10):
                        self.get_logger().info(f"Waiting for drone to reach last waypoint. Measured distance: {distance:.2f} meters")
                        last_print_time = self.get_clock().now()

                if (self.get_clock().now() - start_time) > Duration(seconds=timeout):
                    self.get_logger().error("Timeout waiting for drone to reach the last waypoint.")
                    return False
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
            executor_node.get_logger().error(f"Mission '{mission.get('name', mission['action'])}' failed.")
            success_all = False
            # Depending on requirements, you can choose to continue with next missions or abort.
            # Here, we continue with the next mission.
        else:
            executor_node.get_logger().info(f"Mission '{mission.get('name', mission['action'])}' completed successfully.")

    if success_all:
        executor_node.get_logger().info("All missions executed successfully.")
    else:
        executor_node.get_logger().warn("Some missions failed. Check the logs for details.")

    # Shutdown ROS 2
    executor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
