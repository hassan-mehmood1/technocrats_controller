#!/usr/bin/env python3

import os
import csv
import math
import statistics

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class TechnocratsController(Node):

    def __init__(self):
        super().__init__('technocrats_controller')

        self.declare_parameter(
            'waypoint_file',
            '~/controller_ws/src/technocrats_controller/maps/levine_centerline.csv'
        )

        self.declare_parameter('wheelbase', 0.33)

        self.declare_parameter('cie_controller_gain', 1.0)
        self.declare_parameter('cie_controller_softening', 0.5)

        self.declare_parameter('sensor1_centering_gain', 1.0)
        self.declare_parameter('sensor1_centering_softening', 0.5)

        self.declare_parameter('sensor1_sector_min_angle', 55.0)
        self.declare_parameter('sensor1_sector_max_angle', 125.0)

        self.declare_parameter('min_sensor1_range', 0.10)
        self.declare_parameter('max_sensor1_range', 4.0)

        self.declare_parameter('min_wall_points', 5)
        self.declare_parameter('max_wall_spread', 0.20)

        self.declare_parameter('sensor1_timeout', 0.5)

        self.declare_parameter('front_sector_angle', 12.0)
        self.declare_parameter('front_slow_distance', 1.5)
        self.declare_parameter('front_stop_distance', 0.45)

        self.declare_parameter(
            'max_steering_angle',
            math.radians(24.0)
        )

        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('min_speed', 0.7)
        self.declare_parameter('curvature_speed_gain', 1.5)

        self.declare_parameter('search_forward', 80)
        self.declare_parameter('search_backward', 5)

        self.wheelbase = self.get_parameter('wheelbase').value

        self.cie_controller_gain = (
            self.get_parameter('cie_controller_gain').value
        )

        self.cie_controller_softening = (
            self.get_parameter('cie_controller_softening').value
        )

        self.sensor1_centering_gain = (
            self.get_parameter('sensor1_centering_gain').value
        )

        self.sensor1_centering_softening = (
            self.get_parameter('sensor1_centering_softening').value
        )

        self.sensor1_sector_min_angle = (
            self.get_parameter('sensor1_sector_min_angle').value
        )

        self.sensor1_sector_max_angle = (
            self.get_parameter('sensor1_sector_max_angle').value
        )

        self.min_sensor1_range = (
            self.get_parameter('min_sensor1_range').value
        )

        self.max_sensor1_range = (
            self.get_parameter('max_sensor1_range').value
        )

        self.min_wall_points = (
            self.get_parameter('min_wall_points').value
        )

        self.max_wall_spread = (
            self.get_parameter('max_wall_spread').value
        )

        self.sensor1_timeout = (
            self.get_parameter('sensor1_timeout').value
        )

        self.front_sector_angle = (
            self.get_parameter('front_sector_angle').value
        )

        self.front_slow_distance = (
            self.get_parameter('front_slow_distance').value
        )

        self.front_stop_distance = (
            self.get_parameter('front_stop_distance').value
        )

        self.max_steering_angle = (
            self.get_parameter('max_steering_angle').value
        )

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value

        self.curvature_speed_gain = (
            self.get_parameter('curvature_speed_gain').value
        )

        self.search_forward = (
            self.get_parameter('search_forward').value
        )

        self.search_backward = (
            self.get_parameter('search_backward').value
        )

        waypoint_file = os.path.expanduser(
            self.get_parameter('waypoint_file').value
        )

        self.waypoints = []
        self.track_widths = []

        self.load_waypoints(waypoint_file)

        self.num_waypoints = len(self.waypoints)
        self.previous_index = None

        self.latest_scan = None
        self.last_scan_time = None

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.get_logger().info(
            'TECHNOCRATS CONTROLLER STARTED'
        )

        self.get_logger().info(
            f'Loaded {self.num_waypoints} centreline waypoints'
        )

    def load_waypoints(self, filename):

        if not os.path.isfile(filename):
            raise FileNotFoundError(
                f'Waypoint file not found: {filename}'
            )

        with open(filename, 'r') as file:

            for row in csv.reader(file):

                if not row:
                    continue

                if row[0].strip().startswith('#'):
                    continue

                try:
                    x = float(row[0])
                    y = float(row[1])

                    right_width = float(row[2])
                    left_width = float(row[3])

                    self.waypoints.append((x, y))

                    self.track_widths.append(
                        (right_width, left_width)
                    )

                except (ValueError, IndexError):
                    continue

        if len(self.waypoints) < 3:
            raise RuntimeError(
                'Not enough valid waypoints in CSV file.'
            )

    @staticmethod
    def normalize_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def get_yaw(self, orientation):

        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(sin_yaw, cos_yaw)

    def find_closest_waypoint(self, x, y):

        if self.previous_index is None:

            best_index = 0
            best_distance = float('inf')

            for i, waypoint in enumerate(self.waypoints):

                dx = waypoint[0] - x
                dy = waypoint[1] - y

                distance = dx * dx + dy * dy

                if distance < best_distance:
                    best_distance = distance
                    best_index = i

            self.previous_index = best_index
            return best_index

        best_index = self.previous_index
        best_distance = float('inf')

        for offset in range(
            -self.search_backward,
            self.search_forward + 1
        ):

            index = (
                self.previous_index + offset
            ) % self.num_waypoints

            waypoint = self.waypoints[index]

            dx = waypoint[0] - x
            dy = waypoint[1] - y

            distance = dx * dx + dy * dy

            if distance < best_distance:
                best_distance = distance
                best_index = index

        self.previous_index = best_index

        return best_index

    def cie_controller(self, x, y, yaw, velocity):

        front_x = x + self.wheelbase * math.cos(yaw)
        front_y = y + self.wheelbase * math.sin(yaw)

        closest_index = self.find_closest_waypoint(
            front_x, front_y
        )

        next_index = (
            closest_index + 1
        ) % self.num_waypoints

        current_wp = self.waypoints[closest_index]
        next_wp = self.waypoints[next_index]

        dx = next_wp[0] - current_wp[0]
        dy = next_wp[1] - current_wp[1]

        path_yaw = math.atan2(dy, dx)

        heading_error = self.normalize_angle(
            path_yaw - yaw
        )

        cte = (
            math.cos(path_yaw) *
            (front_y - current_wp[1])
            -
            math.sin(path_yaw) *
            (front_x - current_wp[0])
        )

        speed_for_control = max(
            abs(velocity),
            self.cie_controller_softening
        )

        cross_track_correction = math.atan2(
            self.cie_controller_gain * cte,
            speed_for_control
        )

        steering = (
            heading_error -
            cross_track_correction
        )

        steering = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering)
        )

        curvature = (
            math.tan(steering) / self.wheelbase
        )

        return (
            steering,
            curvature,
            cte,
            heading_error,
            closest_index
        )

    def scan_callback(self, msg):

        self.latest_scan = msg

        self.last_scan_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

    def get_sensor1_points(self):

        if self.latest_scan is None:
            return None

        current_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

        if self.last_scan_time is None:
            return None

        scan_age = current_time - self.last_scan_time

        if scan_age > self.sensor1_timeout:
            return None

        scan = self.latest_scan
        points = []

        angle = scan.angle_min

        for distance in scan.ranges:

            if (
                math.isfinite(distance)
                and self.min_sensor1_range < distance < self.max_sensor1_range
            ):

                px = distance * math.cos(angle)
                py = distance * math.sin(angle)

                points.append((px, py, angle, distance))

            angle += scan.angle_increment

        return points

    def estimate_wall_distance(self, points, side):

        min_angle = math.radians(
            self.sensor1_sector_min_angle
        )

        max_angle = math.radians(
            self.sensor1_sector_max_angle
        )

        lateral_distances = []

        for px, py, angle, distance in points:

            if side == 'left':

                if not (min_angle <= angle <= max_angle):
                    continue

                if py <= 0.0:
                    continue

                lateral_distances.append(py)

            else:

                if not (-max_angle <= angle <= -min_angle):
                    continue

                if py >= 0.0:
                    continue

                lateral_distances.append(abs(py))

        if len(lateral_distances) < self.min_wall_points:
            return None

        wall_distance = statistics.median(
            lateral_distances
        )

        deviations = [
            abs(value - wall_distance)
            for value in lateral_distances
        ]

        spread = statistics.median(deviations)

        if spread > self.max_wall_spread:
            return None

        return wall_distance

    def estimate_both_walls(self):

        points = self.get_sensor1_points()

        if points is None:
            return None, None

        left_distance = self.estimate_wall_distance(
            points, 'left'
        )

        right_distance = self.estimate_wall_distance(
            points, 'right'
        )

        return left_distance, right_distance

    def sensor1_centering_correction(self, velocity):

        left_distance, right_distance = (
            self.estimate_both_walls()
        )

        if left_distance is None or right_distance is None:

            return {
                'correction': 0.0,
                'center_error': None,
                'left_distance': left_distance,
                'right_distance': right_distance,
                'centering_active': False
            }

        center_error = (
            left_distance - right_distance
        ) / 2.0

        speed_for_control = max(
            abs(velocity),
            self.sensor1_centering_softening
        )

        correction = math.atan2(
            self.sensor1_centering_gain * center_error,
            speed_for_control
        )

        return {
            'correction': correction,
            'center_error': center_error,
            'left_distance': left_distance,
            'right_distance': right_distance,
            'centering_active': True
        }

    def get_front_obstacle_distance(self):

        points = self.get_sensor1_points()

        if points is None:
            return None

        front_angle = math.radians(
            self.front_sector_angle
        )

        front_distances = []

        for px, py, angle, distance in points:

            if abs(angle) <= front_angle and px > 0.0:
                front_distances.append(px)

        if not front_distances:
            return None

        front_distances.sort()

        index = max(
            0,
            int(0.10 * (len(front_distances) - 1))
        )

        return front_distances[index]

    def sensor1_cie_controller(self, x, y, yaw, velocity):

        (
            waypoint_steering,
            waypoint_curvature,
            cte,
            heading_error,
            closest_index
        ) = self.cie_controller(
            x, y, yaw, velocity
        )

        sensor1_data = self.sensor1_centering_correction(
            velocity
        )

        sensor1_correction = sensor1_data['correction']

        steering = (
            waypoint_steering +
            sensor1_correction
        )

        steering = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering)
        )

        curvature = (
            math.tan(steering) / self.wheelbase
        )

        front_distance = self.get_front_obstacle_distance()

        diagnostics = {
            'closest_index': closest_index,
            'cte': cte,
            'heading_error': heading_error,
            'waypoint_steering': waypoint_steering,
            'sensor1_correction': sensor1_correction,
            'center_error': sensor1_data['center_error'],
            'left_distance': sensor1_data['left_distance'],
            'right_distance': sensor1_data['right_distance'],
            'centering_active': sensor1_data['centering_active'],
            'front_distance': front_distance
        }

        return steering, curvature, diagnostics

    def calculate_speed(self, curvature, front_distance=None):

        speed = self.max_speed / (
            1.0 +
            self.curvature_speed_gain * abs(curvature)
        )

        speed = max(
            self.min_speed,
            min(self.max_speed, speed)
        )

        if front_distance is not None:

            if front_distance <= self.front_stop_distance:
                return 0.0

            if front_distance < self.front_slow_distance:

                scale = (
                    front_distance - self.front_stop_distance
                ) / (
                    self.front_slow_distance -
                    self.front_stop_distance
                )

                scale = max(0.0, min(1.0, scale))

                speed *= scale

        return speed

    def publish_drive(self, speed, steering):

        msg = AckermannDriveStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/base_link'

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)

        self.drive_pub.publish(msg)

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        yaw = self.get_yaw(
            msg.pose.pose.orientation
        )

        velocity = msg.twist.twist.linear.x

        (
            steering,
            curvature,
            diagnostics
        ) = self.sensor1_cie_controller(
            x, y, yaw, velocity
        )

        front_distance = diagnostics['front_distance']

        center_error = diagnostics['center_error']

        if center_error is None:
            center_text = 'inactive'
        else:
            center_text = f'{center_error:.3f} m'

        left_distance = diagnostics['left_distance']
        right_distance = diagnostics['right_distance']

        left_text = (
            f'{left_distance:.2f} m'
            if left_distance is not None else 'unavailable'
        )

        right_text = (
            f'{right_distance:.2f} m'
            if right_distance is not None else 'unavailable'
        )

        front_text = (
            f'{front_distance:.2f} m'
            if front_distance is not None else 'unavailable'
        )

        speed = self.calculate_speed(
            curvature,
            front_distance
        )

        self.publish_drive(speed, steering)

        diagnostic = (
            f'Closest WP: {diagnostics["closest_index"]} | '
            f'CTE: {diagnostics["cte"]:.3f} m | '
            f'Path Steering: '
            f'{math.degrees(diagnostics["waypoint_steering"]):.2f} deg | '
            f'Sensor1 Correction: '
            f'{math.degrees(diagnostics["sensor1_correction"]):.2f} deg | '
            f'Left: {left_text} | '
            f'Right: {right_text} | '
            f'Center Error: {center_text} | '
            f'Front: {front_text}'
        )

        self.get_logger().info(
            f'Controller: CIE_CONTROLLER | '
            f'Position: ({x:.2f}, {y:.2f}) | '
            f'Yaw: {math.degrees(yaw):.1f} deg | '
            f'{diagnostic} | '
            f'Steering: {math.degrees(steering):.2f} deg | '
            f'Speed: {speed:.2f} m/s',
            throttle_duration_sec=1.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = TechnocratsController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()