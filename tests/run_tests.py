#!/usr/bin/env python3
import os
import subprocess
import sys
import time
import yaml
import argparse
import signal

def load_yaml(file_path):
    """Load a YAML file and return its content."""
    if not os.path.exists(file_path):
        print(f"Error: Configuration file '{file_path}' does not exist.")
        sys.exit(1)
    try:
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data
    except Exception as e:
        print(f"Error: Failed to load YAML file '{file_path}': {e}")
        sys.exit(1)

def substitute_env_variables(config):
    """Substitute environment variables in the configuration."""
    if isinstance(config, dict):
        return {k: substitute_env_variables(v) for k, v in config.items()}
    elif isinstance(config, list):
        return [substitute_env_variables(i) for i in config]
    elif isinstance(config, str) and config.startswith("${") and config.endswith("}"):
        env_var = config[2:-1]
        return os.environ.get(env_var, config)
    else:
        return config

def run_launch_script(script_path, args, cwd):
    """Run a launch script as a subprocess."""
    command = [script_path] + args
    print(f"Running command: {' '.join(command)} in {cwd}")
    process = subprocess.Popen(
        command,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid  # To allow killing the entire process group
    )
    return process

def terminate_process(process):
    """Terminate a subprocess gracefully."""
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5)
        print(f"Process {process.pid} terminated.")
    except Exception as e:
        print(f"Error terminating process {process.pid}: {e}")

def main():
    parser = argparse.ArgumentParser(description='Orchestrate Drone Simulator Tests')
    parser.add_argument('--config', type=str, default='test_suite.yaml', help='Path to the test suite YAML configuration file.')
    args = parser.parse_args()

    # Load and parse the test suite configuration
    config = load_yaml(args.config)
    config = substitute_env_variables(config)

    # Extract configurations
    launch_scripts_dir = config.get('launch_scripts_dir', '.')
    simulation_config = config.get('simulation', {})
    mode_nodes_config = config.get('mode_nodes', {})
    mission_config = config.get('mission', {})
    startup_delay = config.get('startup_delay', 10)

    # Validate launch scripts directory
    if not os.path.isdir(launch_scripts_dir):
        print(f"Error: Launch scripts directory '{launch_scripts_dir}' does not exist.")
        sys.exit(1)

    # Paths to launch scripts
    simulation_script = os.path.join(launch_scripts_dir, simulation_config.get('script'))
    mode_nodes_script = os.path.join(launch_scripts_dir, mode_nodes_config.get('script'))

    # Validate launch scripts
    if not os.path.isfile(simulation_script):
        print(f"Error: Simulation launch script '{simulation_script}' does not exist.")
        sys.exit(1)
    if not os.path.isfile(mode_nodes_script):
        print(f"Error: Mode nodes launch script '{mode_nodes_script}' does not exist.")
        sys.exit(1)

    # Start the simulation
    sim_args = [
        '-f', simulation_config.get('config_file'),
        '-b', simulation_config.get('branch', 'main')
    ]
    simulation_process = run_launch_script(simulation_script, sim_args, launch_scripts_dir)

    # Start the mode nodes
    mode_nodes_args = [
        '-f', mode_nodes_config.get('config_file')
    ]
    mode_nodes_process = run_launch_script(mode_nodes_script, mode_nodes_args, launch_scripts_dir)

    # Sleep to allow simulation and nodes to initialize
    print(f"Sleeping for {startup_delay} seconds to allow initialization...")
    time.sleep(startup_delay)

    # Run the mission tests
    mission_file = mission_config.get('config_file', 'missions.yaml')
    if not os.path.isfile(mission_file):
        print(f"Error: Mission configuration file '{mission_file}' does not exist.")
        # Terminate simulation and nodes before exiting
        terminate_process(simulation_process)
        terminate_process(mode_nodes_process)
        sys.exit(1)

    # Execute the mission tests using the mission_executor.py script
    # Assuming mission_executor.py is in the same directory or in PATH
    mission_executor_script = 'mission_executor.py'
    if not shutil.which(mission_executor_script):
        print(f"Error: '{mission_executor_script}' not found in PATH.")
        # Terminate simulation and nodes before exiting
        terminate_process(simulation_process)
        terminate_process(mode_nodes_process)
        sys.exit(1)

    # Run the mission_executor.py script
    mission_cmd = [
        'python3',
        mission_executor_script,
        '--missions',
        mission_file
    ]
    print(f"Executing mission tests with command: {' '.join(mission_cmd)}")
    mission_process = subprocess.Popen(
        mission_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    # Capture and display mission_executor.py output
    try:
        stdout, stderr = mission_process.communicate()
        if stdout:
            print(stdout.decode())
        if stderr:
            print(stderr.decode())
        if mission_process.returncode != 0:
            print(f"Mission tests failed with return code {mission_process.returncode}")
        else:
            print("Mission tests completed successfully.")
    except KeyboardInterrupt:
        print("Interrupted by user. Terminating processes...")
    finally:
        # Terminate simulation and nodes
        terminate_process(simulation_process)
        terminate_process(mode_nodes_process)

if __name__ == "__main__":
    import shutil
    main()
