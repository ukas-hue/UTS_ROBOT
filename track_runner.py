
import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrackRunner(Node):
    def __init__(self, args):
        super().__init__('track_runner')
        self.args = args
        self.pose = None
        self.last_pose = None
        self.last_record_time = 0.0
        self.points = []
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, args.odom_topic, self.odom_callback, 20
        )
        self.last_log = 0.0
        self.scan = None
        self.scan_time = 0.0
        self.scan_sub = self.create_subscription(
            LaserScan, args.scan_topic, self.scan_callback, 10
        )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = {
            'x': float(p.x),
            'y': float(p.y),
            'yaw': yaw_from_quaternion(q),
            'stamp': time.monotonic(),
        }

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def front_obstacle_distance(self):
        if self.scan is None:
            return None

        if time.monotonic() - self.scan_time > self.args.scan_timeout:
            return None

        msg = self.scan
        if not msg.ranges or msg.angle_increment == 0.0:
            return None

        values = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = wrap(angle)

            if abs(angle) <= math.radians(60):
                if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    values.append(distance)

        return min(values) if values else float('inf')

    def spin_until_scan(self, timeout=5.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and self.scan is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.scan is None:
            raise RuntimeError(
                f"Tidak menerima LiDAR dari {self.args.scan_topic}. "
                "Robot tidak akan dijalankan."
            )

    def spin_until_pose(self, timeout=5.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and self.pose is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            raise RuntimeError(
                f"Tidak menerima odometri dari {self.args.odom_topic}. "
                "Pastikan Gazebo aktif dan topic /odom tersedia."
            )

    def send(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.03)

    def record(self):
        print("\nMODE RECORD: merekam posisi dari /odom.")
        print("Sekarang kendalikan robot dari terminal lain, misalnya wasd_control.py.")
        print("Tekan ENTER di terminal ini untuk mulai merekam.")
        input()
        self.spin_until_pose()
        origin = dict(self.pose)
        self.points = [self.to_local(self.pose, origin)]
        self.last_pose = dict(self.pose)
        self.last_record_time = time.monotonic()
        print("Merekam... tekan ENTER di terminal ini untuk berhenti dan menyimpan.")

        import threading
        finished = threading.Event()

        def wait_enter():
            try:
                input()
            finally:
                finished.set()

        thread = threading.Thread(target=wait_enter, daemon=True)
        thread.start()
        try:
            while rclpy.ok() and not finished.is_set():
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.pose is None:
                    continue
                now = time.monotonic()
                dist = math.hypot(
                    self.pose['x'] - self.last_pose['x'],
                    self.pose['y'] - self.last_pose['y']
                )
                dyaw = abs(wrap(self.pose['yaw'] - self.last_pose['yaw']))

                if (now - self.last_record_time >= self.args.min_interval and
                    (dist >= self.args.record_spacing or dyaw >= self.args.record_yaw)):
                    self.points.append(self.to_local(self.pose, origin))
                    self.last_pose = dict(self.pose)
                    self.last_record_time = now
                    if now - self.last_log > 1.0:
                        print(
                            f"Waypoint {len(self.points)-1}: "
                            f"x={self.points[-1]['x']:.2f} m, "
                            f"y={self.points[-1]['y']:.2f} m, "
                            f"yaw={math.degrees(self.points[-1]['yaw']):.0f}°",
                            flush=True
                        )
                        self.last_log = now
        except KeyboardInterrupt:
            print("\nPerekaman dihentikan dengan Ctrl+C.")
        if self.pose is not None:
            final_point = self.to_local(self.pose, origin)
            if math.hypot(final_point['x'] - self.points[-1]['x'],
                          final_point['y'] - self.points[-1]['y']) > 0.01 or \
                    abs(wrap(final_point['yaw'] - self.points[-1]['yaw'])) > 0.03:
                self.points.append(final_point)
        if len(self.points) < 2:
            raise RuntimeError("Waypoint terlalu sedikit; robot mungkin belum bergerak.")
        data = {
            'format': 'track_runner_route_v1',
            'frame': 'relative_to_recording_start',
            'recorded_at': time.strftime('%Y-%m-%d %H:%M:%S'),
            'points': self.points,
        }
        route_path = os.path.abspath(self.args.route)
        with open(route_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
        print(f"\nTersimpan: {route_path}")
        print(f"Jumlah waypoint: {len(self.points)}")
        print("Simpan posisi awal robot yang sama untuk menjalankan mode coords.")

    @staticmethod
    def to_local(pose, origin):
        dx = pose['x'] - origin['x']
        dy = pose['y'] - origin['y']
        c = math.cos(origin['yaw'])
        s = math.sin(origin['yaw'])
        return {
            'x': c * dx + s * dy,
            'y': -s * dx + c * dy,
            'yaw': wrap(pose['yaw'] - origin['yaw']),
        }

    @staticmethod
    def to_world(point, origin):
        c = math.cos(origin['yaw'])
        s = math.sin(origin['yaw'])
        return {
            'x': origin['x'] + c * point['x'] - s * point['y'],
            'y': origin['y'] + s * point['x'] + c * point['y'],
            'yaw': wrap(origin['yaw'] + point['yaw']),
        }

    def coords(self):
        route_path = os.path.abspath(self.args.route)
        if not os.path.isfile(route_path):
            raise RuntimeError(
                f"File rute belum ada: {route_path}. Jalankan --mode record terlebih dahulu."
            )
        with open(route_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        points = data.get('points', []) if isinstance(data, dict) else data
        if len(points) < 2:
            raise RuntimeError("File rute harus berisi minimal 2 waypoint.")

        self.spin_until_pose()
        origin = dict(self.pose)
        world_points = [self.to_world(p, origin) for p in points]
        print(f"\nMODE COORDS: {len(world_points)} waypoint.")
        print("Pastikan robot berada di titik awal dan menghadap arah yang sama seperti saat merekam.")
        input("Tekan ENTER untuk mulai otomatis, atau Ctrl+C untuk batal.")

        print("MODE OTOMATIS: mengikuti waypoint rekaman WASD.")
        print("Pemeriksaan penghalang LiDAR tidak digunakan pada mode ini.")

        rotation_indices = []
        for i in range(1, len(world_points)):
            prev = world_points[i - 1]
            curr = world_points[i]
            distance = math.hypot(
                curr["x"] - prev["x"],
                curr["y"] - prev["y"]
            )
            yaw_change = abs(wrap(curr["yaw"] - prev["yaw"]))

            if distance < 0.008 and yaw_change > math.radians(2.0):
                rotation_indices.append(i)

        print(f"Waypoint rotasi terdeteksi: {len(rotation_indices)}")
        print("Replay mengikuti urutan waypoint rekaman WASD.")
        print("LiDAR tidak digunakan sebagai pengendali gerak.")

        started = time.monotonic()
        index = 1
        last_progress = time.monotonic()
        best_dist = float("inf")
        last_rotation_error = float("inf")
        last_log = time.monotonic()
        pos_tolerance = min(self.args.pos_tolerance, 0.025)
        rotation_tolerance = math.radians(2.0)
        lookahead_distance = 0.15

        while rclpy.ok() and index < len(world_points):
            for _ in range(3):
                rclpy.spin_once(self, timeout_sec=0.02)

            if self.pose is None:
                continue

            now = time.monotonic()
            robot = self.pose
            target = world_points[index]

            dx = target["x"] - robot["x"]
            dy = target["y"] - robot["y"]
            dist = math.hypot(dx, dy)

            if index in rotation_indices:
                yaw_error = wrap(target["yaw"] - robot["yaw"])

                if abs(yaw_error) <= rotation_tolerance:
                    index += 1
                    best_dist = float("inf")
                    last_rotation_error = float("inf")
                    last_progress = now
                    continue

                if abs(yaw_error) < last_rotation_error - 0.005:
                    last_rotation_error = abs(yaw_error)
                    last_progress = now

                v = 0.0
                w = clamp(
                    1.5 * yaw_error,
                    -self.args.max_angular,
                    self.args.max_angular
                )
                self.send(v, w)

            else:
                if dist <= pos_tolerance:
                    index += 1
                    best_dist = float("inf")
                    last_progress = now
                    continue

                upcoming_rotation = next(
                    (r for r in rotation_indices if r > index),
                    len(world_points) - 1
                )
                limit_index = max(index, upcoming_rotation - 1)

                target_index = index
                path_distance = 0.0

                while target_index < limit_index:
                    a = world_points[target_index]
                    b = world_points[target_index + 1]
                    path_distance += math.hypot(
                        b["x"] - a["x"],
                        b["y"] - a["y"]
                    )
                    target_index += 1

                    if path_distance >= lookahead_distance:
                        break

                target = world_points[target_index]
                dx = target["x"] - robot["x"]
                dy = target["y"] - robot["y"]
                target_dist = math.hypot(dx, dy)

                if target_dist < 0.005:
                    target_heading = robot["yaw"]
                else:
                    target_heading = math.atan2(dy, dx)

                alpha = wrap(target_heading - robot["yaw"])

                if abs(alpha) > 1.0:
                    v = 0.0
                    w = clamp(
                        1.2 * alpha,
                        -self.args.max_angular,
                        self.args.max_angular
                    )
                else:
                    v = self.args.max_linear * max(0.25, math.cos(alpha))
                    w = clamp(
                        2.0 * v * math.sin(alpha) / max(target_dist, 0.08),
                        -self.args.max_angular,
                        self.args.max_angular
                    )

                self.send(v, w)

                if dist < best_dist - 0.01:
                    best_dist = dist
                    last_progress = now

            if now - last_progress > self.args.stuck_seconds:
                raise RuntimeError(
                    f"Waypoint {index} tidak menunjukkan kemajuan; "
                    "robot dihentikan."
                )

            if self.args.max_seconds > 0 and now - started > self.args.max_seconds:
                raise RuntimeError("Batas waktu terlampaui; robot dihentikan.")

            if now - last_log > 1.0:
                print(
                    f"WP {index}/{len(world_points)-1} "
                    f"| posisi=({robot['x']:.2f},{robot['y']:.2f}) "
                    f"| sisa ke titik={dist:.2f} m",
                    flush=True
                )
                last_log = now

        self.stop()
        print(f"Rute selesai dalam {time.monotonic()-started:.1f} detik.")
        print("Verifikasi secara visual bahwa robot mencapai titik akhir.")

def parse_args():
    parser = argparse.ArgumentParser(
        description="Rekam lintasan robot dari odom dan ulangi secara otomatis."
    )
    parser.add_argument('--mode', choices=['record', 'coords'], required=True)
    parser.add_argument('--route', default='rute_robot.json')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--scan-topic', default='/robin/scan')
    parser.add_argument('--obstacle-stop', type=float, default=0.30,
                        help='jarak minimum objek di depan dalam meter')
    parser.add_argument('--scan-timeout', type=float, default=2.0,
                        help='batas usia data LiDAR dalam detik')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--record-spacing', type=float, default=0.06, help='jarak rekam minimum (m)')
    parser.add_argument('--record-yaw', type=float, default=0.12, help='perubahan yaw minimum (rad)')
    parser.add_argument('--min-interval', type=float, default=0.12, help='interval rekam minimum (s)')
    parser.add_argument('--max-linear', type=float, default=0.07, help='kecepatan maksimum m/s')
    parser.add_argument('--max-angular', type=float, default=0.35, help='kecepatan putar maksimum rad/s')
    parser.add_argument('--pos-tolerance', type=float, default=0.07, help='toleransi posisi m')
    parser.add_argument('--yaw-tolerance', type=float, default=0.20, help='toleransi orientasi rad')
    parser.add_argument('--rotate-first-angle', type=float, default=0.35, help='sudut putar sebelum maju rad')
    parser.add_argument('--stuck-seconds', type=float, default=12.0)
    parser.add_argument('--max-seconds', type=float, default=600.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = TrackRunner(args)
    code = 0
    try:
        if args.mode == 'record':
            node.record()
        else:
            node.coords()
    except KeyboardInterrupt:
        print("\nDihentikan pengguna.")
    except (RuntimeError, ValueError, OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        code = 1
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
