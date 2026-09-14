#!/usr/bin/env python
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from pykinisi import *
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import quaternion_from_euler


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
        # goBILDA 5202 series, 50.9:1 planetary gearbox: 28 CPR (encoder shaft)
        # x 50.9 = 1425.1 counts per output-shaft (wheel) revolution. Confirmed
        # empirically: hand-rotating a wheel 5 revs read 7137 ticks (1427/rev).
        encoder_resolution = 1425.1

        if platform_type == "omni":
            self.kinisi_controller.initialize_omni_platform(
                is_reversed_0=True,
                is_reversed_1=True,
                is_reversed_2=True,
                is_encoder_reversed_0=False,
                is_encoder_reversed_1=False,
                is_encoder_reversed_2=False,
                wheels_diameter=0.096, # 9.6 cm
                robot_radius=0.185, # 0.175 m
                encoder_resolution=encoder_resolution
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
                encoder_resolution=encoder_resolution
            )
        else:
            print("Unknown platform type")
            exit()

        # Set three platform velocity components
        self.kinisi_controller.start_platform_controller(
            kp=0.8, # Proportional gain
            ki=0.2, # Integral gain
            kd=0, # Derivative gain
            integral_limit=30 # Absolute maximum value of integral value.
        )

        self.kinisi_controller.start_platform_odometry()
        self.kinisi_controller.reset_platform_odometry()

        # Indicate that platform is ready
        self.kinisi_controller.toggle_status_led_state()
        time.sleep(0.5) # in seconds
        self.kinisi_controller.toggle_status_led_state()
        
        # Enable tf2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Previous odometry sample (x, y, yaw, t_sec) for finite-difference twist.
        self._last_odom = None

        # Enable odom publishing
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        odom_publish_period = 0.1  # 10 Hz
        self.odom_timer = self.create_timer(odom_publish_period, self.odom_publish_callback)

        # Enable subscriber for velocity commands
        self.cmd_vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Enable subscriber for odometry reset requests
        self.reset_odom_subscriber = self.create_subscription(Empty, "/reset_odometry", self.reset_odometry_callback, 10)
        self.get_logger().info('Started kinisi_controller node.')

    def reset_odometry_callback(self, msg):
        self.get_logger().info('Resetting platform odometry.')
        self.kinisi_controller.reset_platform_odometry()

    def odom_publish_callback(self):
        odometry_data = self.kinisi_controller.get_platform_odometry()

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        # Firmware reports world-frame pose directly (it rotates each body-frame
        # increment by heading before accumulating), so publish x/y as-is.
        x = odometry_data.x
        y = odometry_data.y
        yaw = odometry_data.t

        # Estimate the velocity by finite-differencing the pose. REP-105 requires
        # the Odometry twist to be expressed in the child frame (base_link), so
        # rotate the world-frame linear velocity back into the body frame.
        # NOTE: angular.z MUST be an angular *velocity*. Previously this field was
        # (incorrectly) set to the heading angle itself, which made Nav2/DWB think
        # the robot was already spinning ever faster and drove a runaway rotation.
        vx = 0.0
        vy = 0.0
        wz = 0.0
        if self._last_odom is not None:
            lx, ly, lyaw, lt = self._last_odom
            dt = now_sec - lt
            if dt > 1e-6:
                dx = (x - lx) / dt
                dy = (y - ly) / dt
                dyaw = math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw)) / dt
                c = math.cos(yaw)
                s = math.sin(yaw)
                vx = dx * c + dy * s
                vy = -dx * s + dy * c
                wz = dyaw
        self._last_odom = (x, y, yaw, now_sec)

        # Convert yaw to quaternion (shared by the Odometry pose and the TF).
        q = quaternion_from_euler(0, 0, yaw)  # roll, pitch, yaw

        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = now.to_msg()
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = wz
        self.odom_publisher.publish(odom_msg)

        # Publish tf2
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        

    def cmd_vel_callback(self, msg):
        self.get_logger().debug('Received: "%s"' % msg)
        self.kinisi_controller.set_platform_target_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

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