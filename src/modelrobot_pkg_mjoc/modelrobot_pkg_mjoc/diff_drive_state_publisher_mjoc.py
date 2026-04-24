#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState # referencia

class DiffDriveStatePublisher_mjoc(Node): # <--- CHANGE ME
    def __init__(self):
        super().__init__("diff_drive_state_publisher_mjoc") # <--- CHANGE ME
        # ---- Parameters (declare + get) ----
        self.declare_parameter('left_wheel_joint', 'bl2lw')
        self.declare_parameter('right_wheel_joint', 'bl2rw')
        self.declare_parameter('publish_rate', 50.0)      # Hz

        self.l_joint = self.get_parameter('left_wheel_joint').get_parameter_value().string_value
        self.r_joint = self.get_parameter('right_wheel_joint').get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter('publish_rate').value)

        # ---- State ----
        self.cmd_w1 = 0.0
        self.cmd_w2 = 0.0 

        # wheel angles (rad)
        self.theta_l = 0.0
        self.theta_r = 0.0

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, qos)
        self.pub_js = self.create_publisher(JointState, '/joint_states', qos)

        # ---- Timer ----
        self.last_time = None
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f"Joints: {self.l_joint}, {self.r_joint}, rate={self.rate_hz} Hz\n"
        )
    def _on_cmd_vel(self, msg: Twist):
    self.cmd_w1 = msg.angular.x
    self.cmd_w2 = msg.angular.y

    def _on_timer(self):
    now = self.get_clock().now()
    if self.last_time is None:
        self.last_time = now
        return

    dt = (now - self.last_time).nanoseconds * 1e-9
    self.last_time = now
    if dt <= 0.0:
        return

    # wheel angular velocities
    w_r = self.cmd_w1
    w_l = self.cmd_w2

    # integrate wheel angles
    self.theta_r += w_r * dt
    self.theta_l += w_l * dt


    # Publish joint_states 
    js = JointState()
    js.header.stamp = now.to_msg()
    js.name = [self.l_joint, self.r_joint]
    js.position = [self.theta_l, self.theta_r]
    js.velocity = [w_l, w_r]
    self.pub_js.publish(js)

def main():
    rclpy.init()
    node = DiffDriveStatePublisher_mjoc() # <--- CHANGE ME
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()