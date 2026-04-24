#!/usr/bin/env python3
"""
diff_drive_simulator.py
"""

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, Vector3, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState          
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time

# Es como ROS Trabaja con Quaterniones
def quaternion_from_yaw(yaw: float) -> tuple:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class DiffDriveSimulator(Node):

    def __init__(self):
        super().__init__('diff_drive_simulator')

        # ── Parámetros del robot ──────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.025)  # radio de la rueda es en metros 
        self.declare_parameter('wheel_base',   0.0925) # distancia entre ruedas
        self.declare_parameter('update_rate',  50.0)   # ciclos por segundo
        self.declare_parameter('max_wheel_vel', 0.5)   # Velocidad máxima por rueda 

        self.R    = self.get_parameter('wheel_radius').value
        self.L    = self.get_parameter('wheel_base').value
        self.rate = self.get_parameter('update_rate').value
        self.vmax = self.get_parameter('max_wheel_vel').value

        # ── Estado del robot ──────────────────────────────────────────────
        self.x  = 0.0   # posicion del robot(body_x)
        self.y  = 0.0   # posicion del robot(body_y)
        self.th = 0.0

        self.v_left  = 0.0  # Velocidades lineales de cada rueda (m/s)
        self.v_right = 0.0

        # ── Ángulos acumulados de las ruedas ──────────────────────────────
        self.angle_left  = 0.0    # Ángulo acumulado de cada rueda (rad)
        self.angle_right = 0.0          

        # ── Publicadores ──────────────────────────────────────────────────
        self.pub_odom   = self.create_publisher(Odometry,          '/odom',        10)    #Posición + velocidad completa
        self.pub_pose   = self.create_publisher(Pose2D,            '/robot_pose',  10)    #informacion del robot 
        self.pub_wheels = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)   #Velocidades de ruedas
        self.pub_joints = self.create_publisher(JointState,        '/joint_states', 10)   #Ángulos/vel para RViz
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Suscriptores ──────────────────────────────────────────────────
        self.create_subscription(
            Vector3, '/wheel_velocities', self._cb_wheel_vel, 10)  #Comando directo a ruedas
        self.create_subscription(
            Twist, '/cmd_vel', self._cb_cmd_vel, 10)               #Comando de velocidad del cuerpo

        # ── Timer ─────────────────────────────────────────────────────────
        dt = 1.0 / self.rate
        self.create_timer(dt, self._update)
        self._last_time = self.get_clock().now()

        self.get_logger().info(
            f'DiffDriveSimulator iniciado — L={self.L}m  R={self.R}m  '
            f'rate={self.rate}Hz'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cb_wheel_vel(self, msg: Vector3):   # Recibe directamente v_izq y v_der
        self.v_left  = max(-self.vmax, min(self.vmax, msg.x))
        self.v_right = max(-self.vmax, min(self.vmax, msg.y))

    def _cb_cmd_vel(self, msg: Twist):   # Recibe velocidad del cuerpo y hace CINEMÁTICA INVERSA  body->wheels
        v = msg.linear.x
        w = msg.angular.z
        self.v_left  = max(-self.vmax, min(self.vmax, v - (w * self.L) / 2.0))
        self.v_right = max(-self.vmax, min(self.vmax, v + (w * self.L) / 2.0))

    # ── Ciclo principal ────────────────────────────────────────────────────

    def _update(self):
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt <= 0.0:
            return

        # ── CINEMÁTICA DIRECTA ─────────────────────────────────────────   wheels -> body
        v = (self.v_right + self.v_left) / 2.0       # Velocidad lineal del robot
        w = (self.v_right - self.v_left) / self.L    # Velocidad angular del robot

        # ── INTEGRACIÓN EULER ──────────────────────────────────────────
        self.th += w * dt                                             # Actualiza orientación
        self.th  = math.atan2(math.sin(self.th), math.cos(self.th))   # Normaliza a [-π, π]
        self.x  += v * math.cos(self.th) * dt                         # Actualiza posición X
        self.y  += v * math.sin(self.th) * dt                         # Actualiza posición Y

        # ── ÁNGULO DE LAS RUEDAS ─────────────────────────────────────── (RViz)
        self.angle_left  += (self.v_left  / self.R) * dt               # ω_rueda = v / R 
        self.angle_right += (self.v_right / self.R) * dt   

        # ── PUBLICAR ───────────────────────────────────────────────────
        self._publish_odom(now, v, w)
        self._publish_pose()
        self._publish_wheel_speeds()
        self._publish_joint_states(now)    
        self._broadcast_tf(now)

    def _publish_odom(self, now, v: float, w: float):
        qx, qy, qz, qw = quaternion_from_yaw(self.th)
        msg = Odometry()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = 'odom'        #frame padre (mundo)
        msg.child_frame_id  = 'base_link'   # frame del robot
        # POSE — dónde está el robot
        msg.pose.pose.position.x    = self.x
        msg.pose.pose.position.y    = self.y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # TWIST — qué tan rápido se mueve
        msg.twist.twist.linear.x    = v
        msg.twist.twist.angular.z   = w
        self.pub_odom.publish(msg)

    def _publish_pose(self):  #Pose simplificada
        msg = Pose2D()
        msg.x     = self.x
        msg.y     = self.y
        msg.theta = self.th
        self.pub_pose.publish(msg)

    def _publish_wheel_speeds(self): #Velocidades de ruedas
        msg = Float32MultiArray()
        msg.data = [float(self.v_left), float(self.v_right)]
        self.pub_wheels.publish(msg)

    def _publish_joint_states(self, now):  # Estado de las articulaciones     # para la aniamcion de rviz     
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name     = ['chassis_to_wheel_left', 'chassis_to_wheel_right']
        msg.position = [self.angle_left,          self.angle_right]
        msg.velocity = [self.v_left  / self.R,    self.v_right / self.R]
        self.pub_joints.publish(msg)

    def _broadcast_tf(self, now):  # Árbol de transformaciones
        qx, qy, qz, qw = quaternion_from_yaw(self.th)
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()