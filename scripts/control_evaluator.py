#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ControlEvaluator(Node):
    """Evaluate trajectory tracking and write one report after the mission."""

    def __init__(self):
        super().__init__('control_evaluator')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('start_delay', 2.0)
        self.declare_parameter('position_tolerance', 0.25)
        self.declare_parameter('report_file', '/tmp/quadcopter_control_report.txt')
        self.declare_parameter('controller_name', 'unknown')

        mission_file = self.get_parameter('mission_file').value
        if not mission_file:
            raise ValueError('Parameter mission_file must not be empty')
        with open(mission_file, encoding='utf-8') as stream:
            mission = yaml.safe_load(stream)
        points = mission.get('waypoints', [])
        if len(points) < 2:
            raise ValueError('Mission requires at least two waypoints')

        self.mission_file = mission_file
        self.mission_duration = float(points[-1]['t'])
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.expected_duration = self.start_delay + self.mission_duration
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.report_file = Path(
            self.get_parameter('report_file').value).expanduser()
        self.controller_name = self.get_parameter('controller_name').value
        self.reference_points = [
            (float(p['x']), float(p['y']), float(p['z'])) for p in points]
        self.segment_lengths = [
            math.dist(a, b)
            for a, b in zip(self.reference_points, self.reference_points[1:])]
        self.reference_length = sum(self.segment_lengths)

        self.target = None
        self.start_time = None
        self.last_time = None
        self.last_position = None
        self.report_written = False

        self.duration = 0.0
        self.samples = 0
        self.sum_path_error = 0.0
        self.sum_path_error_sq = 0.0
        self.max_path_error = 0.0
        self.time_in_tolerance = 0.0
        self.path_length = 0.0
        self.max_route_progress = 0.0
        self.final_path_error = math.nan
        self.final_goal_error = math.nan

        self.create_subscription(PoseStamped, 'command/pose', self.target_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 20)
        self.get_logger().info(
            f'Evaluator ready; report after {self.expected_duration:.2f} s of mission time')

    def target_cb(self, msg):
        self.target = msg
        if self.start_time is None:
            self.start_time = self.get_clock().now()

    def odom_cb(self, msg):
        if self.report_written or self.target is None or self.start_time is None:
            return

        now = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_time is None:
            self.last_time = now
            self.last_position = self.position(msg)
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.1:
            self.last_position = self.position(msg)
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        position = self.position(msg)
        if elapsed < self.start_delay:
            self.last_position = position
            return

        path_error, route_progress = self.distance_to_reference_path(position)
        self.duration += dt
        self.samples += 1
        self.sum_path_error += path_error * dt
        self.sum_path_error_sq += path_error * path_error * dt
        self.max_path_error = max(self.max_path_error, path_error)
        self.max_route_progress = max(self.max_route_progress, route_progress)
        if path_error <= self.position_tolerance:
            self.time_in_tolerance += dt

        if self.last_position is not None:
            self.path_length += math.dist(position, self.last_position)
        self.last_position = position

        self.final_path_error = path_error
        self.final_goal_error = math.dist(position, self.reference_points[-1])

        if elapsed >= self.expected_duration:
            self.write_report(elapsed)

    @staticmethod
    def position(msg):
        p = msg.pose.pose.position
        return p.x, p.y, p.z

    def distance_to_reference_path(self, position):
        """Return 3D distance and arc-length progress to the nearest path segment."""
        best_distance = math.inf
        best_progress = 0.0
        progress_before_segment = 0.0
        for start, end, length in zip(
                self.reference_points, self.reference_points[1:], self.segment_lengths):
            if length <= 1e-12:
                progress_before_segment += length
                continue
            segment = tuple(end[i] - start[i] for i in range(3))
            relative = tuple(position[i] - start[i] for i in range(3))
            fraction = sum(relative[i] * segment[i] for i in range(3)) / (length * length)
            fraction = min(1.0, max(0.0, fraction))
            nearest = tuple(start[i] + fraction * segment[i] for i in range(3))
            distance = math.dist(position, nearest)
            if distance < best_distance:
                best_distance = distance
                best_progress = progress_before_segment + fraction * length
            progress_before_segment += length
        return best_distance, best_progress

    def write_report(self, elapsed):
        self.report_written = True
        if self.duration <= 0.0:
            self.get_logger().error('Cannot create report: no valid odometry samples')
            return

        path_rmse = math.sqrt(self.sum_path_error_sq / self.duration)
        tolerance_percent = 100.0 * self.time_in_tolerance / self.duration
        coverage_percent = (100.0 * self.max_route_progress / self.reference_length
                            if self.reference_length > 0.0 else 100.0)
        report = f"""RAPORT OCENY STEROWANIA QUADROTOREM
=====================================
Regulator: {self.controller_name}
Misja: {self.mission_file}
Czas nominalny aktywnego lotu: {self.mission_duration:.3f} s
Czas ocenianych pomiarow: {self.duration:.3f} s
Liczba probek odometrii: {self.samples}

DOKLADNOSC PODAZANIA ZA TRASA
----------------------------
RMSE odleglosci od trasy: {path_rmse:.4f} m
Srednia odleglosc od trasy: {self.sum_path_error / self.duration:.4f} m
Maksymalna odleglosc od trasy: {self.max_path_error:.4f} m
Koncowa odleglosc od trasy: {self.final_path_error:.4f} m
Odleglosc od koncowego punktu misji: {self.final_goal_error:.4f} m
Czas w korytarzu trasy (odleglosc <= {self.position_tolerance:.3f} m): {tolerance_percent:.2f} %
Pokonana czesc trasy: {coverage_percent:.2f} %
Dlugosc trasy referencyjnej: {self.reference_length:.4f} m
Dlugosc rzeczywistego lotu: {self.path_length:.4f} m
"""
        try:
            self.report_file.parent.mkdir(parents=True, exist_ok=True)
            self.report_file.write_text(report, encoding='utf-8')
        except OSError as error:
            self.get_logger().error(f'Nie mozna zapisac raportu: {error}')
        self.get_logger().info('\n' + report)
        self.get_logger().info(f'Raport zapisany w: {self.report_file}')


def main():
    rclpy.init()
    node = ControlEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
