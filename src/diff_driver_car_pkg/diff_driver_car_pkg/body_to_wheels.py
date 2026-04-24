#!/usr/bin/env python3
"""
body_to_wheels.py
=================
Nodo que muestra la cinemática INVERSA del carro diferencial.

Convierte velocidades del cuerpo (v, ω) a velocidades de rueda (v_izq, v_der)

CINEMÁTICA INVERSA:
  Entradas : v  = velocidad lineal  del cuerpo [m/s]
             ω  = velocidad angular del cuerpo [rad/s]

  Salidas  : v_izq = v - (ω · L) / 2   [m/s]
             v_der = v + (ω · L) / 2   [m/s]

TÓPICOS:
  Suscribe:  /cmd_vel        (geometry_msgs/Twist)   v y ω del cuerpo
  Publica:   /wheel_velocities (geometry_msgs/Vector3) x=v_izq, y=v_der
             /ik_debug        (std_msgs/String)       info para ver en terminal
"""

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String


class BodyToWheels(Node):
    """
    Convierte velocidades del cuerpo a velocidades de rueda.

    Parámetros:
      wheel_base     : distancia L entre ruedas [m]
      wheel_radius   : radio de las ruedas [m]
      max_wheel_vel  : velocidad máxima de rueda [m/s]
    """

    def __init__(self):
        super().__init__('body_to_wheels')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('wheel_base',    0.0925)
        self.declare_parameter('wheel_radius',  0.025)
        self.declare_parameter('max_wheel_vel', 0.5)

        self.L    = self.get_parameter('wheel_base').value
        self.R    = self.get_parameter('wheel_radius').value
        self.vmax = self.get_parameter('max_wheel_vel').value

        # ── Publicadores ──────────────────────────────────────────────────
        self.pub_wheels = self.create_publisher(
            Vector3, '/wheel_velocities', 10)
        self.pub_debug  = self.create_publisher(
            String, '/ik_debug', 10)

        # ── Suscriptor ────────────────────────────────────────────────────
        self.create_subscription(
            Twist, '/cmd_vel', self._cb_cmd_vel, 10)

        self.get_logger().info(
            f'BodyToWheels iniciado — L={self.L}m  R={self.R}m'
        )
        self.get_logger().info(
            'Enviando comandos a /cmd_vel para ver la conversión'
        )

    def _cb_cmd_vel(self, msg: Twist):
        """
        Recibe velocidad del cuerpo y calcula velocidades de rueda.

        CINEMÁTICA INVERSA:
          v_izq = v - (ω · L) / 2
          v_der = v + (ω · L) / 2
        """
        v = msg.linear.x    # velocidad lineal  [m/s]
        w = msg.angular.z   # velocidad angular [rad/s]

        # ── CINEMÁTICA INVERSA ────────────────────────────────────────────
        v_izq = v - (w * self.L) / 2.0
        v_der = v + (w * self.L) / 2.0

        # ── Convertir a velocidad angular de rueda [rad/s] ─────────────
        w_izq = v_izq / self.R
        w_der = v_der / self.R

        # ── Saturar velocidades ───────────────────────────────────────────
        v_izq_sat = max(-self.vmax, min(self.vmax, v_izq))
        v_der_sat = max(-self.vmax, min(self.vmax, v_der))

        # ── Detectar tipo de movimiento ───────────────────────────────────
        tipo = self._tipo_movimiento(v, w)

        # ── Publicar velocidades de rueda ─────────────────────────────────
        wheels_msg = Vector3()
        wheels_msg.x = v_izq_sat   # rueda izquierda
        wheels_msg.y = v_der_sat   # rueda derecha
        self.pub_wheels.publish(wheels_msg)

        # ── Publicar debug ────────────────────────────────────────────────
        debug = String()
        debug.data = (
            f'\n{"="*45}'
            f'\n  TIPO     : {tipo}'
            f'\n  ENTRADA  : v={v:.3f} m/s   ω={w:.3f} rad/s'
            f'\n{"─"*45}'
            f'\n  CINEMÁTICA INVERSA:'
            f'\n  v_izq = v - (ω·L)/2 = {v:.3f} - ({w:.3f}·{self.L})/2 = {v_izq:.3f} m/s'
            f'\n  v_der = v + (ω·L)/2 = {v:.3f} + ({w:.3f}·{self.L})/2 = {v_der:.3f} m/s'
            f'\n{"─"*45}'
            f'\n  VELOCIDAD ANGULAR DE RUEDA:'
            f'\n  ω_izq = v_izq / R = {v_izq:.3f} / {self.R} = {w_izq:.3f} rad/s'
            f'\n  ω_der = v_der / R = {v_der:.3f} / {self.R} = {w_der:.3f} rad/s'
            f'\n{"─"*45}'
            f'\n  SALIDA (saturada): v_izq={v_izq_sat:.3f}  v_der={v_der_sat:.3f}'
            f'\n{"="*45}'
        )
        self.pub_debug.publish(debug)

        # También imprimir en terminal
        self.get_logger().info(
            f'{tipo} | v={v:.2f} ω={w:.2f} → '
            f'v_izq={v_izq:.3f}  v_der={v_der:.3f}  '
            f'(ω_izq={w_izq:.1f} ω_der={w_der:.1f} rad/s)'
        )

    def _tipo_movimiento(self, v: float, w: float) -> str:
        """Detecta el tipo de movimiento según v y ω."""
        if abs(v) < 0.001 and abs(w) < 0.001:
            return 'QUIETO          '
        elif abs(w) < 0.001:
            return 'LINEA RECTA     ' if v > 0 else 'RETROCESO       '
        elif abs(v) < 0.001:
            return 'GIRO EN SITIO   '
        elif v > 0 and w > 0:
            return 'ARCO IZQ        '
        elif v > 0 and w < 0:
            return 'ARCO DER        '
        else:
            return 'MOVIMIENTO MIXTO'


def main(args=None):
    rclpy.init(args=args)
    node = BodyToWheels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()