import time
import socket  # use raw Bluetooth socket, no pybluez

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class BTCarBridge(Node):
    def __init__(self):
        super().__init__('bt_car_bridge')

        # Parameters
        self.declare_parameter('hc05_mac', '98:D3:C1:FE:9C:45')
        self.declare_parameter('max_pwm', 200)
        self.declare_parameter('wheel_base', 0.2)  # meters, rough

        self.hc05_mac = self.get_parameter('hc05_mac').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Bluetooth socket (AF_BLUETOOTH + RFCOMM)
        self.sock = None
        self._connect_bt()

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.get_logger().info(
            f"BTCarBridge started. HC-05={self.hc05_mac}, "
            f"max_pwm={self.max_pwm}, wheel_base={self.wheel_base}"
        )

    def _connect_bt(self):
        """Establish RFCOMM connection to HC-05 using raw socket."""
        self.get_logger().info(
            f"Connecting to HC-05 at {self.hc05_mac} (RFCOMM channel 1)..."
        )

        # Some Python builds don't define BTPROTO_RFCOMM constant, default to 3
        BTPROTO_RFCOMM = getattr(socket, "BTPROTO_RFCOMM", 3)

        try:
            s = socket.socket(socket.AF_BLUETOOTH,
                              socket.SOCK_STREAM,
                              BTPROTO_RFCOMM)
            s.connect((self.hc05_mac, 1))  # channel 1 = SPP
            self.sock = s
            self.get_logger().info("Bluetooth connected.")
            time.sleep(0.2)  # small settle delay
        except Exception as e:
            self.get_logger().error(f"Failed to connect to HC-05: {e}")
            self.sock = None

    def _send_cmd(self, left_pwm: int, right_pwm: int):
        """Send 'V <left> <right>' over Bluetooth."""
        if self.sock is None:
            self.get_logger().warn("No BT socket; trying to reconnect...")
            self._connect_bt()
            if self.sock is None:
                self.get_logger().error("Reconnect failed; dropping command.")
                return

        cmd = f"V {left_pwm} {right_pwm}\n"
        try:
            self.sock.send(cmd.encode("ascii"))
        except Exception as e:
            self.get_logger().error(f"Bluetooth send failed: {e}")
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None  # mark for reconnect next time

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x      # m/s
        w = msg.angular.z     # rad/s

        L = self.wheel_base

        # Diff-drive kinematics
        v_left = v - (w * L / 2.0)
        v_right = v + (w * L / 2.0)

        # Map to PWM
        v_max = 1.0  # assume 1 m/s => max_pwm
        scale = self.max_pwm / v_max

        left_pwm = int(max(-self.max_pwm, min(self.max_pwm, v_left * scale)))
        right_pwm = int(max(-self.max_pwm, min(self.max_pwm, v_right * scale)))

        # self.get_logger().info(f"cmd_vel -> V {left_pwm} {right_pwm}")
        self._send_cmd(left_pwm, right_pwm)


def main(args=None):
    rclpy.init(args=args)
    node = BTCarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock is not None:
            try:
                node.sock.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

