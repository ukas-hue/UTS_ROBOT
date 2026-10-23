
import json
import math
import os
import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ROUTE = os.path.expanduser("~/rute_robot_manual.json")
LINEAR = 0.08
ANGULAR = 0.40


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Recorder(Node):
    def __init__(self):
        super().__init__("manual_recorder")
        self.pose = None
        self.origin = None
        self.points = []
        self.last = None
        self.last_time = 0.0
        self.v = 0.0
        self.w = 0.0

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = {
            "x": p.x,
            "y": p.y,
            "yaw": yaw_of(msg.pose.pose.orientation)
        }

    def local_pose(self):
        p = self.pose
        o = self.origin
        dx, dy = p["x"] - o["x"], p["y"] - o["y"]
        c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
        return {
            "x": c*dx + s*dy,
            "y": -s*dx + c*dy,
            "yaw": wrap(p["yaw"] - o["yaw"])
        }

    def record(self, force=False):
        if self.pose is None or self.origin is None:
            return

        p = self.local_pose()
        now = time.monotonic()

        if self.last is None:
            self.points.append(p)
            self.last = p
            self.last_time = now
            return

        dist = math.hypot(
            p["x"] - self.last["x"],
            p["y"] - self.last["y"]
        )
        dyaw = abs(wrap(p["yaw"] - self.last["yaw"]))

        if force or (
            now - self.last_time >= 0.12
            and (dist >= 0.04 or dyaw >= 0.10)
        ):
            self.points.append(p)
            self.last = p
            self.last_time = now

    def save(self):
        if len(self.points) < 2:
            print("\nRute belum cukup panjang.")
            return

        if os.path.exists(ROUTE):
            backup = ROUTE + "." + time.strftime("%Y%m%d_%H%M%S") + ".bak"
            os.replace(ROUTE, backup)
            print("Rute sebelumnya dicadangkan:", backup)

        with open(ROUTE, "w") as f:
            json.dump({
                "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "points": self.points
            }, f, indent=2)

        print(f"\nRute tersimpan: {ROUTE}")
        print("Jumlah waypoint:", len(self.points))


def main():
    if not sys.stdin.isatty():
        print("Jalankan langsung dari terminal.")
        return

    rclpy.init()
    node = Recorder()
    old = termios.tcgetattr(sys.stdin)

    try:
        deadline = time.monotonic() + 10
        while rclpy.ok() and node.pose is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > deadline:
                raise RuntimeError("Odometri /odom tidak tersedia.")

        node.origin = dict(node.pose)
        node.record()

        print("\n=== ROBIN MANUAL + REKAM ===")
        print("W maju | S mundur | A kiri | D kanan")
        print("SPACE berhenti | P simpan | Q simpan dan keluar")
        print("Kecepatan:", LINEAR, "m/s")
        print("Pastikan robot bebas dari dinding dan penghalang.")

        tty.setraw(sys.stdin.fileno())

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                key = sys.stdin.read(1).lower()

                if key == "w":
                    node.v, node.w = LINEAR, 0.0
                elif key == "s":
                    node.v, node.w = -LINEAR, 0.0
                elif key == "a":
                    node.v, node.w = 0.0, ANGULAR
                elif key == "d":
                    node.v, node.w = 0.0, -ANGULAR
                elif key == " ":
                    node.v, node.w = 0.0, 0.0
                elif key == "p":
                    node.v, node.w = 0.0, 0.0
                    node.save()
                elif key == "q":
                    node.v, node.w = 0.0, 0.0
                    node.record(force=True)
                    node.save()
                    break

            cmd = Twist()
            cmd.linear.x = node.v
            cmd.angular.z = node.w
            node.pub.publish(cmd)
            node.record()

    except KeyboardInterrupt:
        node.v, node.w = 0.0, 0.0
        node.record(force=True)
        node.save()
    except Exception as e:
        print("\nERROR:", e)
    finally:
        for _ in range(5):
            node.pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
