//WASD control
  W = maju, S = mundur, A = putar kiri, D = putar kanan
  X atau spasi = berhenti
  R = mulai rekaman / akhiri dan simpan rekaman
  Q = keluar

Hasil rekaman default: ~/rute_robot.json
Setiap waypoint berisi x, y, yaw relatif terhadap posisi awal saat R ditekan.
"""

import json
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LINEAR_SPEED = 0.10
ANGULAR_SPEED = 0.65
INPUT_TIMEOUT = 0.80
PUBLISH_PERIOD = 0.05
CMD_TOPIC = '/cmd_vel'
ODOM_TOPIC = '/odom'
ROUTE_FILE = os.path.expanduser('~/rute_robot.json')
RECORD_DISTANCE = 0.06
RECORD_ANGLE = 0.12
RECORD_INTERVAL = 0.12


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def local_pose(pose, origin):
    dx = pose['x'] - origin['x']
    dy = pose['y'] - origin['y']
    c = math.cos(origin['yaw'])
    s = math.sin(origin['yaw'])
    return {
        'x': c * dx + s * dy,
        'y': -s * dx + c * dy,
        'yaw': wrap(pose['yaw'] - origin['yaw']),
    }


def send_stop(pub):
    msg = Twist()
    pub.publish(msg)


def save_route(points):
    if len(points) < 2:
        print('\nRekaman tidak disimpan: robot belum cukup bergerak (waypoint < 2).')
        return
    payload = {
        'format': 'robot_route_v1',
        'frame': 'relative_to_recording_start',
        'saved_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        'odom_topic': ODOM_TOPIC,
        'points': points,
    }
    path = os.path.abspath(ROUTE_FILE)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2)
    print(f'\nLintasan tersimpan: {path}')
    print(f'Jumlah waypoint: {len(points)}')


def main():
    if not sys.stdin.isatty():
        raise RuntimeError('Jalankan dari terminal Ubuntu interaktif.')

    rclpy.init()
    node = rclpy.create_node('keyboard_wasd_controller')
    pub = node.create_publisher(Twist, CMD_TOPIC, 10)

    state = {
        'pose': None, 'recording': False, 'origin': None,
        'points': [], 'last_pose': None, 'last_sample': 0.0,
    }

    def odom_cb(msg):
        p = msg.pose.pose.position
        state['pose'] = {
            'x': float(p.x),
            'y': float(p.y),
            'yaw': yaw_from_quaternion(msg.pose.pose.orientation),
        }

    node.create_subscription(Odometry, ODOM_TOPIC, odom_cb, 20)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    linear = angular = 0.0
    last_key_time = time.monotonic()

    print('\n=== KONTROL WASD + REKAM LINTASAN ===')
    print('W maju | S mundur | A kiri | D kanan | X/spasi berhenti')
    print('R mulai/akhiri rekaman | Q keluar')
    print(f'Data posisi: {ODOM_TOPIC} | Perintah gerak: {CMD_TOPIC}')
    print(f'File hasil: {ROUTE_FILE}')
    print('Pastikan Gazebo aktif. Tekan R di titik awal sebelum menggerakkan robot.')

    def add_final_point():
        pose = state['pose']
        if pose is None or state['origin'] is None:
            return
        point = local_pose(pose, state['origin'])
        if not state['points']:
            state['points'].append(point)
            return
        last = state['points'][-1]
        if (math.hypot(point['x'] - last['x'], point['y'] - last['y']) > 0.01 or
                abs(wrap(point['yaw'] - last['yaw'])) > 0.03):
            state['points'].append(point)

    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            if select.select([sys.stdin], [], [], PUBLISH_PERIOD)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'w':
                    linear, angular = LINEAR_SPEED, 0.0
                elif key == 's':
                    linear, angular = -LINEAR_SPEED, 0.0
                elif key == 'a':
                    linear, angular = 0.0, ANGULAR_SPEED
                elif key == 'd':
                    linear, angular = 0.0, -ANGULAR_SPEED
                elif key in ('x', ' '):
                    linear = angular = 0.0
                elif key == 'r':
                    linear = angular = 0.0
                    if not state['recording']:
                        if state['pose'] is None:
                            print(f'\nBelum menerima {ODOM_TOPIC}. Pastikan Gazebo berjalan.')
                        else:
                            state['origin'] = dict(state['pose'])
                            state['points'] = [{'x': 0.0, 'y': 0.0, 'yaw': 0.0}]
                            state['last_pose'] = dict(state['pose'])
                            state['last_sample'] = time.monotonic()
                            state['recording'] = True
                            print('\nREKAMAN DIMULAI. Gerakkan robot mengikuti lintasan.')
                    else:
                        add_final_point()
                        state['recording'] = False
                        save_route(state['points'])
                        print('Rekaman berhenti. Tekan R untuk merekam lintasan baru.')
                elif key in ('q', '\x03'):
                    break
                else:
                    continue
                last_key_time = time.monotonic()

            if time.monotonic() - last_key_time > INPUT_TIMEOUT:
                linear = angular = 0.0

            cmd = Twist()
            cmd.linear.x = float(linear)
            cmd.angular.z = float(angular)
            pub.publish(cmd)

            if state['recording'] and state['pose'] is not None:
                now = time.monotonic()
                cur = state['pose']
                prev = state['last_pose']
                distance = math.hypot(cur['x'] - prev['x'], cur['y'] - prev['y'])
                turn = abs(wrap(cur['yaw'] - prev['yaw']))
                if now - state['last_sample'] >= RECORD_INTERVAL and (
                    distance >= RECORD_DISTANCE or turn >= RECORD_ANGLE
                ):
                    point = local_pose(cur, state['origin'])
                    state['points'].append(point)
                    state['last_pose'] = dict(cur)
                    state['last_sample'] = now
                    print(
                        f"\rWaypoint {len(state['points'])-1}: "
                        f"x={point['x']:.2f} m y={point['y']:.2f} m "
                        f"yaw={math.degrees(point['yaw']):.0f}°   ",
                        end='', flush=True
                    )
    except KeyboardInterrupt:
        pass
    finally:
        send_stop(pub)
        time.sleep(0.1)
        if state['recording']:
            add_final_point()
            save_route(state['points'])
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\nRobot dihentikan. Program selesai.')


if __name__ == '__main__':
    main()
