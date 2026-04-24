#!/usr/bin/env python3
"""
goto_point.py
=============
Nodo ROS 2 que navega el carro diferencial a un punto (gx, gy).

ESTRATEGIA (control proporcional simple):
  1. Calcular el ángulo hacia el objetivo:  α = atan2(gy-y, gx-x)
  2. Error angular:  e_θ = α - θ  (normalizado a -π..π)
  3. Distancia:      d   = sqrt((gx-x)² + (gy-y)²)

  Si |e_θ| > umbral_giro → girar primero (v=0, ω = Kp_ω · e_θ)
  Si no                  → avanzar     (v = Kp_v · d, ω = Kp_ω · e_θ)

  Robot se detiene cuando d < tolerancia_dist

TÓPICOS:
  Suscribe:  /robot_pose  (geometry_msgs/Pose2D)
  Publica:   /cmd_vel     (geometry_msgs/Twist)
"""

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, Pose2D


class GotoPoint(Node):
    """
    Controlador P para navegar a un punto objetivo.

    Parámetros:
      goal_x, goal_y  : coordenadas del punto objetivo [m]
      kp_linear       : ganancia proporcional lineal
      kp_angular      : ganancia proporcional angular
      dist_tolerance  : tolerancia de llegada [m]
      angle_threshold : umbral para girar antes de avanzar [rad]
      max_linear_vel  : velocidad lineal máxima [m/s]
      max_angular_vel : velocidad angular máxima [rad/s]
    """

    def __init__(self):
        super().__init__('goto_point')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('goal_x',          2.0)
        self.declare_parameter('goal_y',          1.5)
        self.declare_parameter('kp_linear',       0.5)
        self.declare_parameter('kp_angular',      2.0)
        self.declare_parameter('dist_tolerance',  0.05)
        self.declare_parameter('angle_threshold', 0.15)   # ~8.6°
        self.declare_parameter('max_linear_vel',  0.5)
        self.declare_parameter('max_angular_vel', 1.5)

        self.gx   = self.get_parameter('goal_x').value
        self.gy   = self.get_parameter('goal_y').value
        self.kp_v = self.get_parameter('kp_linear').value
        self.kp_w = self.get_parameter('kp_angular').value
        self.tol  = self.get_parameter('dist_tolerance').value
        self.a_th = self.get_parameter('angle_threshold').value
        self.vmax = self.get_parameter('max_linear_vel').value
        self.wmax = self.get_parameter('max_angular_vel').value

        # Estado
        self.pose    = Pose2D()
        self.reached = False

        # Suscriptores y publicadores
        self.create_subscription(Pose2D, '/robot_pose', self._cb_pose, 10)  # el se suscribe al topic que sale del diff_driver
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)         

        # Controlador a 20 Hz
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'GotoPoint iniciado — objetivo: ({self.gx:.2f}, {self.gy:.2f}) m'
        )

    def _cb_pose(self, msg: Pose2D):
        self.pose = msg

    def _control_loop(self):
        if self.reached:
            return

        x, y, th = self.pose.x, self.pose.y, self.pose.theta

        # ── Errores ───────────────────────────────────────────────────────────
        dx = self.gx - x
        dy = self.gy - y
        dist = math.hypot(dx, dy) #distancia euclidiana al objetivo -sqrt(dx² + dy²)

        # Ángulo hacia el objetivo en el marco global
        alpha = math.atan2(dy, dx)

        # Error angular normalizado a [-π, π]
        e_th = alpha - th
        e_th = math.atan2(math.sin(e_th), math.cos(e_th))

        cmd = Twist()

        if dist < self.tol:
            # ── Llegó al objetivo ─────────────────────────────────────────────
            self.reached = True
            self.get_logger().info(
                f'¡Objetivo alcanzado! Posición final: '
                f'x={x:.3f} y={y:.3f} θ={math.degrees(th):.1f}°'
            )
        elif abs(e_th) > self.a_th:
            # ── Fase 1: orientarse hacia el objetivo ──────────────────────────
            cmd.angular.z = max(-self.wmax, min(self.wmax, self.kp_w * e_th))
            cmd.linear.x  = 0.0
        else:
            # ── Fase 2: avanzar con corrección angular ────────────────────────
            cmd.linear.x  = max(0.0, min(self.vmax, self.kp_v * dist))
            cmd.angular.z = max(-self.wmax, min(self.wmax, self.kp_w * e_th))

        self.pub_cmd.publish(cmd)

        # Log periódico (cada ~1s = cada 20 ciclos aprox.)
        if int(self.get_clock().now().nanoseconds / 5e8) % 2 == 0:
            self.get_logger().info(
                f'd={dist:.2f}m  e_θ={math.degrees(e_th):.1f}°  '
                f'v={cmd.linear.x:.2f}  ω={cmd.angular.z:.2f}',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = GotoPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
