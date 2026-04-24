#!/usr/bin/env python3
"""
keyboard_teleop.py
==================
Teleoperación simple por teclado para el carro diferencial.

  W / S  : avanzar / retroceder
  A / D  : girar izquierda / derecha
  Espacio: freno de emergencia
  Q      : salir
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios


KEY_BINDINGS = {
    'w': ( 0.2,  0.0),   # avanzar
    's': (-0.2,  0.0),   # retroceder
    'a': ( 0.0,  0.5),   # girar izquierda
    'd': ( 0.0, -0.5),   # girar derecha
    ' ': ( 0.0,  0.0),   # parar
}


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info(
            'Teleoperación activa\n'
            '  W/S : avanzar/retroceder\n'
            '  A/D : girar izquierda/derecha\n'
            '  SPC : parar\n'
            '  Q   : salir'
        )

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        v, w = 0.0, 0.0
        while rclpy.ok():
            key = self.get_key().lower()
            if key == 'q':
                break
            if key in KEY_BINDINGS:
                v, w = KEY_BINDINGS[key]
            cmd = Twist()
            cmd.linear.x  = v
            cmd.angular.z = w
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
