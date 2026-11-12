#!/usr/bin/env python3

import os
import csv
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class TechnocratsController(Node):

    def __init__(self):
        super().__init__('technocrats_controller')

        # =====================================================
        # PARAMETERS
        # =====================================================

        self.declare_parameter(
            'first_lap_file',
            '~/controller_ws/src/technocrats_controller/maps/second_lap.csv'
        )

        self.declare_parameter(
            'other_lap_file',
            '~/controller_ws/src/technocrats_controller/maps/third_lap.csv'
        )

        self.declare_parameter('wheelbase', 0.33)

        # cie controller
        self.declare_parameter('cie_gain', 1.0)
        self.declare_parameter('cie_softening', 0.5)

        # Vehicle constraints
        self.declare_parameter(
            'max_steering_angle',
            math.radians(24.0)
        )

        # Speed control
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('min_speed', 0.7)
        self.declare_parameter('curvature_speed_gain', 1.5)

        # Waypoint search
        self.declare_parameter('search_forward', 80)
        self.declare_parameter('search_backward', 5)

        # Lap detection
        self.declare_parameter('lap_wrap_high_fraction', 0.80)
        self.declare_parameter('lap_wrap_low_fraction', 0.20)

        # =====================================================
        # sensor1 PARAMETERS
        # =====================================================

        self.declare_parameter('sensor1_topic', '/scan')

        self.declare_parameter('avoidance_distance', 0.22)

        self.declare_parameter(
            'avoidance_steering_angle',
            math.radians(20.0)
        )

        self.declare_parameter('avoidance_speed', 0.45)

        self.declare_parameter('side_sector_start', 8.0)
        self.declare_parameter('side_sector_end', 60.0)

        self.declare_parameter('sensor1_timeout', 0.5)

        # =====================================================
        # LOAD PARAMETERS
        # =====================================================

        self.wheelbase = self.get_parameter('wheelbase').value

        self.cie_gain = self.get_parameter('cie_gain').value
        self.cie_softening = self.get_parameter(
            'cie_softening'
        ).value

        self.max_steering_angle = self.get_parameter(
            'max_steering_angle'
        ).value

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value

        self.curvature_speed_gain = self.get_parameter(
            'curvature_speed_gain'
        ).value

        self.search_forward = self.get_parameter(
            'search_forward'
        ).value

        self.search_backward = self.get_parameter(
            'search_backward'
        ).value
        
        self.lap_wrap_high_fraction = self.get_parameter(
            'lap_wrap_high_fraction'
        ).value

        self.lap_wrap_low_fraction = self.get_parameter(
            'lap_wrap_low_fraction'
        ).value

        # sensor1 parameters
        self.avoidance_distance = self.get_parameter(
            'avoidance_distance'
        ).value

        self.avoidance_steering_angle = self.get_parameter(
            'avoidance_steering_angle'
        ).value

        self.avoidance_speed = self.get_parameter(
            'avoidance_speed'
        ).value

        self.side_sector_start = self.get_parameter(
            'side_sector_start'
        ).value

        self.side_sector_end = self.get_parameter(
            'side_sector_end'
        ).value

        self.sensor1_timeout = self.get_parameter(
            'sensor1_timeout'
        ).value

        sensor1_topic = self.get_parameter('sensor1_topic').value

        # =====================================================
        # LAP MANAGEMENT
        # =====================================================

        self.first_lap_file = os.path.expanduser(
            self.get_parameter('first_lap_file').value
        )

        self.other_lap_file = os.path.expanduser(
            self.get_parameter('other_lap_file').value
        )

        self.current_lap = 1

        # The first path is loaded at startup.
        self.active_waypoint_file = self.first_lap_file

        self.waypoints = []
        self.track_widths = []

        self.previous_index = None
        self.last_progress_index = None

        self.load_waypoints(self.active_waypoint_file)

        self.num_waypoints = len(self.waypoints)

        # =====================================================
        # sensor1 DATA
        # =====================================================

        self.latest_scan = None
        self.last_scan_time = None

        # =====================================================
        # ROS 2 INTERFACES
        # =====================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            sensor1_topic,
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # =====================================================
        # STARTUP INFORMATION
        # =====================================================

        self.get_logger().info('========================================')
        self.get_logger().info('TECHNOCRATS cie + SIDE sensor1')
        self.get_logger().info('AUTOMATIC LAP-BASED MAP SWITCHING')
        self.get_logger().info('========================================')

        self.get_logger().info(
            f'First lap map: {self.first_lap_file}'
        )

        self.get_logger().info(
            f'Other laps map: {self.other_lap_file}'
        )

        self.get_logger().info(
            f'Active map: {self.active_waypoint_file}'
        )

        self.get_logger().info(
            f'Waypoints: {self.num_waypoints}'
        )

        self.get_logger().info(
            f'Avoidance distance: {self.avoidance_distance} m'
        )

        self.get_logger().info(
            f'Avoidance steering: '
            f'{math.degrees(self.avoidance_steering_angle):.1f} deg'
        )

        self.get_logger().info(
            f'Avoidance speed: {self.avoidance_speed} m/s'
        )

        self.get_logger().info(
            'Front obstacle detection: DISABLED'
        )

        self.get_logger().info(
            'Starting on LAP 1'
        )

        self.get_logger().info('========================================')

    # =========================================================
    # LOAD WAYPOINTS
    # =========================================================

    def load_waypoints(self, filename):

        if not os.path.isfile(filename):
            raise FileNotFoundError(
                f'Waypoint file not found: {filename}'
            )

        new_waypoints = []
        new_track_widths = []

        with open(filename, 'r') as file:

            reader = csv.reader(file)

            for row in reader:

                if not row:
                    continue

                if row[0].strip().startswith('#'):
                    continue

                try:

                    x = float(row[0])
                    y = float(row[1])

                    right_width = float(row[2])
                    left_width = float(row[3])

                    if not all(map(math.isfinite, [
                        x,
                        y,
                        right_width,
                        left_width
                    ])):
                        continue

                    new_waypoints.append((x, y))

                    new_track_widths.append(
                        (right_width, left_width)
                    )

                except (ValueError, IndexError):
                    continue

        if len(new_waypoints) < 3:
            raise RuntimeError(
                f'Not enough valid waypoints in {filename}'
            )

        # Replace active path only after successful loading.
        self.waypoints = new_waypoints
        self.track_widths = new_track_widths

        self.num_waypoints = len(self.waypoints)

        self.previous_index = None
        self.last_progress_index = None

        self.get_logger().info(
            f'Loaded {self.num_waypoints} waypoints from {filename}'
        )

    # =========================================================
    # SWITCH TO OTHER LAP MAP
    # =========================================================

    def switch_to_other_lap_map(self, x, y, yaw):

        if self.current_lap >= 2:
            return

        self.get_logger().warn(
            '========================================'
        )

        self.get_logger().warn(
            'FIRST LAP COMPLETED!'
        )

        self.get_logger().warn(
            'Switching to OTHER_LAP.csv'
        )

        # Change active lap.
        self.current_lap = 2

        self.active_waypoint_file = self.other_lap_file

        # Load the new path.
        self.load_waypoints(self.active_waypoint_file)

        # Find the closest waypoint on the new path.
        front_x = (
            x + self.wheelbase * math.cos(yaw)
        )

        front_y = (
            y + self.wheelbase * math.sin(yaw)
        )

        best_index = 0
        best_distance = float('inf')

        for i, waypoint in enumerate(self.waypoints):

            dx = waypoint[0] - front_x
            dy = waypoint[1] - front_y

            distance = dx * dx + dy * dy

            if distance < best_distance:

                best_distance = distance
                best_index = i

        self.previous_index = best_index

        self.last_progress_index = best_index

        self.get_logger().warn(
            f'NOW USING OTHER_LAP.csv | '
            f'Waypoints: {self.num_waypoints} | '
            f'Closest waypoint: {best_index}'
        )

        self.get_logger().warn(
            'All remaining laps will use OTHER_LAP.csv'
        )

        self.get_logger().warn(
            '========================================'
        )

    # =========================================================
    # LAP COMPLETION DETECTION
    # =========================================================

    def check_lap_completion(self, closest_index, x, y, yaw):

        if self.last_progress_index is None:

            self.last_progress_index = closest_index
            return

        high_threshold = int(
            self.lap_wrap_high_fraction * self.num_waypoints
        )

        low_threshold = int(
            self.lap_wrap_low_fraction * self.num_waypoints
        )

        previous_index = self.last_progress_index

        # Detect progress wrapping from the end of the path
        # back to its beginning.
        lap_completed = (
            previous_index >= high_threshold
            and closest_index <= low_threshold
        )

        if lap_completed:

            self.get_logger().info(
                f'Lap boundary detected: '
                f'{previous_index} -> {closest_index}'
            )

            if self.current_lap == 1:

                self.switch_to_other_lap_map(
                    x,
                    y,
                    yaw
                )

                return

            else:

                self.current_lap += 1

                self.get_logger().info(
                    f'Lap {self.current_lap} started | '
                    f'Using OTHER_LAP.csv'
                )

        self.last_progress_index = closest_index

    # =========================================================
    # ANGLE NORMALIZATION
    # =========================================================

    @staticmethod
    def normalize_angle(angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    # =========================================================
    # GET VEHICLE YAW
    # =========================================================

    def get_yaw(self, orientation):

        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        sin_yaw = 2.0 * (w * z + x * y)

        cos_yaw = (
            1.0 -
            2.0 * (y * y + z * z)
        )

        return math.atan2(sin_yaw, cos_yaw)

    # =========================================================
    # FIND CLOSEST WAYPOINT
    # =========================================================

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

    # =========================================================
    # PURE cie CONTROLLER
    # =========================================================

    def cie_controller(self, x, y, yaw, velocity):

        front_x = (
            x +
            self.wheelbase * math.cos(yaw)
        )

        front_y = (
            y +
            self.wheelbase * math.sin(yaw)
        )

        closest_index = self.find_closest_waypoint(
            front_x,
            front_y
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

        speed_for_control = (
            abs(velocity) +
            self.cie_softening
        )

        cross_track_correction = math.atan2(
            self.cie_gain * cte,
            speed_for_control
        )

        steering = (
            heading_error -
            cross_track_correction
        )

        steering = max(
            -self.max_steering_angle,
            min(
                self.max_steering_angle,
                steering
            )
        )

        curvature = (
            math.tan(steering) /
            self.wheelbase
        )

        return (
            steering,
            curvature,
            cte,
            heading_error,
            cross_track_correction,
            closest_index
        )

    # =========================================================
    # sensor1 CALLBACK
    # =========================================================

    def scan_callback(self, msg):

        self.latest_scan = msg

        self.last_scan_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

    # =========================================================
    # GET MINIMUM DISTANCE IN SECTOR
    # =========================================================

    def get_sector_minimum(self, scan, angle_start, angle_end):

        minimum_distance = float('inf')

        for i, distance in enumerate(scan.ranges):

            angle = (
                scan.angle_min +
                i * scan.angle_increment
            )

            angle_deg = math.degrees(angle)

            if not (
                angle_start <= angle_deg <= angle_end
            ):
                continue

            if not math.isfinite(distance):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                continue

            minimum_distance = min(
                minimum_distance,
                distance
            )

        return minimum_distance

    # =========================================================
    # sensor1 SIDE-OBSTACLE AVOIDANCE
    # =========================================================

    def sensor1_avoidance(self):

        if self.latest_scan is None:
            return False, 0.0, 0.0, 'NO sensor1 DATA'

        current_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

        scan_age = current_time - self.last_scan_time

        if scan_age > self.sensor1_timeout:
            return False, 0.0, 0.0, 'STALE sensor1'

        scan = self.latest_scan

        # Left sector: positive angles
        left_min = self.get_sector_minimum(
            scan,
            self.side_sector_start,
            self.side_sector_end
        )

        # Right sector: negative angles
        right_min = self.get_sector_minimum(
            scan,
            -self.side_sector_end,
            -self.side_sector_start
        )

        left_blocked = (
            left_min < self.avoidance_distance
        )

        right_blocked = (
            right_min < self.avoidance_distance
        )

        # Both sides blocked: use cie
        if left_blocked and right_blocked:

            return (
                False,
                0.0,
                0.0,
                'BOTH SIDES BLOCKED - cie'
            )

        # Obstacle on left: steer right
        if left_blocked:

            return (
                True,
                -self.avoidance_steering_angle,
                self.avoidance_speed,
                f'OBSTACLE LEFT ({left_min:.2f} m) - AVOID RIGHT'
            )

        # Obstacle on right: steer left
        if right_blocked:

            return (
                True,
                self.avoidance_steering_angle,
                self.avoidance_speed,
                f'OBSTACLE RIGHT ({right_min:.2f} m) - AVOID LEFT'
            )

        # No side obstacle
        return (
            False,
            0.0,
            0.0,
            'SIDES CLEAR - cie'
        )

    # =========================================================
    # SPEED CONTROL
    # =========================================================

    def calculate_speed(self, curvature):

        speed = self.max_speed / (
            1.0 +
            self.curvature_speed_gain * abs(curvature)
        )

        speed = max(
            self.min_speed,
            min(self.max_speed, speed)
        )

        return speed

    # =========================================================
    # PUBLISH DRIVE COMMAND
    # =========================================================

    def publish_drive(self, speed, steering):

        msg = AckermannDriveStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = (
            'ego_racecar/base_link'
        )

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)

        self.drive_pub.publish(msg)

    # =========================================================
    # ODOMETRY CALLBACK
    # =========================================================

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        yaw = self.get_yaw(
            msg.pose.pose.orientation
        )

        velocity = msg.twist.twist.linear.x

        # -----------------------------------------------------
        # cie CONTROL
        # -----------------------------------------------------

        (
            cie_steering,
            curvature,
            cte,
            heading_error,
            cross_track_correction,
            closest_index
        ) = self.cie_controller(
            x,
            y,
            yaw,
            velocity
        )

        # -----------------------------------------------------
        # LAP DETECTION
        # -----------------------------------------------------

        self.check_lap_completion(
            closest_index,
            x,
            y,
            yaw
        )

        # -----------------------------------------------------
        # SPEED CONTROL
        # -----------------------------------------------------

        cie_speed = self.calculate_speed(curvature)

        # -----------------------------------------------------
        # sensor1 AVOIDANCE
        # -----------------------------------------------------

        (
            avoidance_active,
            avoidance_steering,
            avoidance_speed,
            avoidance_reason
        ) = self.sensor1_avoidance()

        # -----------------------------------------------------
        # FINAL CONTROL SELECTION
        # -----------------------------------------------------

        if avoidance_active:

            final_steering = avoidance_steering
            final_speed = avoidance_speed

            controller_mode = 'sensor1 AVOIDANCE'

        else:

            final_steering = cie_steering
            final_speed = cie_speed

            controller_mode = 'PURE cie'

        # -----------------------------------------------------
        # PUBLISH DRIVE COMMAND
        # -----------------------------------------------------

        self.publish_drive(
            final_speed,
            final_steering
        )

        # -----------------------------------------------------
        # DIAGNOSTICS
        # -----------------------------------------------------

        self.get_logger().info(

            f'Lap: {self.current_lap} | '

            f'Map: {os.path.basename(self.active_waypoint_file)} | '

            f'Controller: {controller_mode} | '

            f'Position: ({x:.2f}, {y:.2f}) | '

            f'Yaw: {math.degrees(yaw):.2f} deg | '

            f'Closest WP: {closest_index} | '

            f'CTE: {cte:.3f} m | '

            f'cie Steering: '
            f'{math.degrees(cie_steering):.2f} deg | '

            f'Final Steering: '
            f'{math.degrees(final_steering):.2f} deg | '

            f'Speed: {final_speed:.2f} m/s | '

            f'{avoidance_reason}',

            throttle_duration_sec=1.0
        )


# =============================================================
# MAIN
# =============================================================

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

