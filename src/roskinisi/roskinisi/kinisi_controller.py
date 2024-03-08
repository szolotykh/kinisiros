#!/usr/bin/env python
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pykinisi import *


class KinisiControllerNode(Node):
    def __init__(self):
        super().__init__('kinisi_controller')

        # Declare parameters
        self.declare_parameter('port', 'NOT_SET')

        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value

        # Check if the port parameter was set
        if port == 'NOT_SET':
            self.get_logger().error('No port specified. Please set the "port" parameter.')
            rclpy.shutdown()
            return

        # Enable kinisi controller
        self.kinisi_controller = KinisiController()

        # Connect to the controller
        if not self.kinisi_controller.connect(port):
            self.get_logger().error(f"Can't open serial connection with the controller. Port: {port}")
            rclpy.shutdown()
            return

        # Initialize platform
        platform_type = "omni"

        if platform_type == "omni":
            self.kinisi_controller.initialize_omni_platform(
                is_reversed_0=False,
                is_reversed_1=False,
                is_reversed_2=False,
                wheels_diameter=0.1, # 10 cm
                robot_radius=0.15, # 15 cm
                encoder_resolution=0
            )
        elif platform_type == "mecanum":
            self.kinisi_controller.initialize_mecanum_platform(
                is_reversed_0=False,
                is_reversed_1=False,
                is_reversed_2=False,
                is_reversed_3=False,
                length= 0.5, # 50 cm
                width= 0.4, # 40 cm
                wheels_diameter=0.1, # 10 cm
                encoder_resolution=0
            )
        else:
            print("Unknown platform type")
            exit()

        # Set three platform velocity components
        self.kinisi_controller.start_platform_controller(
            kp=1, # Proportional gain
            ki=0.1, # Integral gain
            kd=0, # Derivative gain
            integral_limit=30 # Absolute maximum value of integral value.
        )

        # Indicate that platform is ready
        self.kinisi_controller.toggle_status_led_state()
        time.sleep(0.5) # in seconds
        self.kinisi_controller.toggle_status_led_state()
        
        # Enable odom publishing
        self.odom_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #odom_publish_period = 1  # seconds
        #self.odom_timer = self.create_timer(odom_publish_period, self.odom_publish_callback)

        # Enable subscriber for velocity commands
        self.cmd_vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.get_logger().info('Started kinisi_controller node.')

    def odom_publish_callback(self):
        pass

    def cmd_vel_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg)
        self.kinisi_controller.set_platform_target_velocity(msg.linear.x, msg.linear.y, msg.angular.z)
        self.kinisi_controller.toggle_status_led_state()

    def __del__(self):
        self.kinisi_controller.stop_platform_controller()
        self.get_logger().info('Kinisi controller disconnected.')

def main(args=None):
    rclpy.init(args=args)
    kinisi_controller = KinisiControllerNode()

    if rclpy.ok():
        rclpy.spin(kinisi_controller)
        kinisi_controller.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()