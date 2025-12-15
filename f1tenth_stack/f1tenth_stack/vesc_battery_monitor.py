#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped
import subprocess
import os


class VescBatteryMonitor(Node):
    """
    Monitor VESC battery voltage and shut down system when voltage drops below threshold.
    Monitors /sensors/core topic and reads state.voltage_input field.
    """

    def __init__(self):
        super().__init__('vesc_battery_monitor')
        
        # Declare parameters - actual values should be set in vesc.yaml
        # These are just fallback defaults if YAML is not provided
        self.declare_parameter('voltage_threshold', 0.0)
        self.declare_parameter('check_duration', 1.0)
        self.declare_parameter('warning_threshold', 0.0)
        self.declare_parameter('shutdown_command', 'sudo shutdown -h now')
        self.declare_parameter('sudo_password', '')
        
        # Get parameters (loaded from vesc.yaml via launch file)
        self.voltage_threshold = self.get_parameter('voltage_threshold').get_parameter_value().double_value
        self.check_duration = self.get_parameter('check_duration').get_parameter_value().double_value
        self.warning_threshold = self.get_parameter('warning_threshold').get_parameter_value().double_value
        self.shutdown_command = self.get_parameter('shutdown_command').get_parameter_value().string_value
        self.sudo_password = self.get_parameter('sudo_password').get_parameter_value().string_value
        
        # State tracking
        self.low_voltage_start_time = None
        self.last_voltage = None
        self.warning_issued = False
        self.shutdown_initiated = False
        
        # Subscribe to VESC state
        self.vesc_state_sub = self.create_subscription(
            VescStateStamped,
            '/sensors/core',
            self.vesc_state_callback,
            10
        )
        
        self.get_logger().info(
            f'Starting voltage monitor (threshold: {self.voltage_threshold}V)'
        )
        self.get_logger().info(f'Monitoring topic: /sensors/core')
        self.get_logger().info(f'Check duration: {self.check_duration}s')
        self.get_logger().info('Press Ctrl+C to stop')

    def vesc_state_callback(self, msg):
        """Process VESC state messages and monitor voltage."""
        if self.shutdown_initiated:
            return
            
        voltage = msg.state.voltage_input
        self.last_voltage = voltage
        current_time = self.get_clock().now()
        
        # Log current voltage reading
        self.get_logger().info(f'Current voltage: {voltage:.2f}V')
        
        # Check if voltage is critically low
        if voltage < self.voltage_threshold:
            if self.low_voltage_start_time is None:
                # First time below threshold
                self.low_voltage_start_time = current_time
                self.get_logger().warn(
                    f'WARNING: Voltage {voltage:.2f}V is below threshold {self.voltage_threshold}V'
                )
                self.get_logger().warn(
                    f'Initiating clean shutdown in {self.check_duration} seconds...'
                )
            else:
                # Check how long we've been below threshold
                duration = (current_time - self.low_voltage_start_time).nanoseconds / 1e9
                
                if duration >= self.check_duration:
                    self.get_logger().error('Shutting down system...')
                    self.shutdown_initiated = True
                    self.shutdown_system()
        else:
            # Voltage recovered above threshold
            if self.low_voltage_start_time is not None:
                self.get_logger().info(
                    f'Battery voltage recovered to {voltage:.2f}V. Shutdown cancelled.'
                )
            self.low_voltage_start_time = None
            
            # Check for warning threshold
            if voltage < self.warning_threshold and not self.warning_issued:
                self.get_logger().warn(
                    f'Battery voltage ({voltage:.2f}V) below warning threshold '
                    f'({self.warning_threshold}V). Consider charging soon.'
                )
                self.warning_issued = True
            elif voltage >= self.warning_threshold:
                self.warning_issued = False

    def shutdown_system(self):
        """Execute system shutdown command"""
        try:
            if self.sudo_password:
                # Use sudo with password 
                command = f'echo {self.sudo_password} | sudo -S {self.shutdown_command.replace("sudo ", "")}'
                self.get_logger().info(f'Executing shutdown with sudo password')
            else:
                # Execute shutdown command as-is
                command = self.shutdown_command
                self.get_logger().info(f'Executing shutdown command: {self.shutdown_command}')
            
            subprocess.run(command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to execute shutdown command: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error during shutdown: {e}')
        finally:
            # Attempt ROS shutdown as fallback
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    battery_monitor = VescBatteryMonitor()
    
    try:
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        battery_monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

