
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AutoRobot(Node):

    def __init__(self):
        super().__init__('auto_robot')

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/robin/scan',
            self.scan_callback,
            10
        )

        self.scan = None
        self.start_time = None
        self.duration = 7.85
        self.finished = False

        self.timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info(
            'Pengujian putaran kiri dimulai.'
        )

    def scan_callback(self, msg):
        self.scan = msg

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.finished:
            self.stop_robot()
            return

        if self.scan is None:
            self.stop_robot()
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if self.start_time is None:
            self.start_time = now

        elapsed = now - self.start_time

        if elapsed >= self.duration:
            self.stop_robot()
            self.finished = True

            self.get_logger().info(
                'Pengujian selesai. Robot diperintahkan berhenti.'
            )

            self.timer.cancel()
            return

        cmd = Twist()
        cmd.angular.z = 0.2
        self.cmd_pub.publish(cmd)

        sectors = {}

        for i, r in enumerate(self.scan.ranges):
            angle = math.degrees(
                self.scan.angle_min
                + i * self.scan.angle_increment
            )

            if not math.isfinite(r):
                continue

            if not self.scan.range_min <= r <= self.scan.range_max:
                continue

            sector = int(round(angle / 15.0)) * 15
            sectors.setdefault(sector, []).append(r)

        if sectors:
            nearest = min(
                (min(values), angle)
                for angle, values in sectors.items()
            )

            self.get_logger().info(
                f'Waktu={elapsed:.1f}s | '
                f'Pantulan terdekat: sudut={nearest[1]}°, '
                f'jarak={nearest[0]:.2f} m',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = AutoRobot()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
