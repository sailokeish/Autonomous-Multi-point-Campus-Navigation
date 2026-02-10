#!/usr/bin/env python3
"""Christofides algorithm for robot campus tour planning."""

import argparse
import math
import os
import time
import xml.etree.ElementTree as ET
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# Waypoint format: (name, x, y)
Waypoint = tuple[str, float, float]

DEFAULT_WORLD_FILE = "northeastern_campus_scaled.world"
DEFAULT_PREFIX = "marker_"

MARKER_NAMES = {
    'marker_QUAD': 'ORIGIN',
    'marker_DG': 'DODGE',
    'marker_EL': 'ELL',
    'marker_RH': 'RICHARDS',
    'marker_CSC': 'CURRY',
    'marker_SL': 'SNELL_LIB',
    'marker_CH': 'CHURCHILL',
    'marker_SE': 'SNELL_ENG'
}

# Hand-tuned coordinates to keep robot away from walls
SAFE_POSITIONS = {
    'RICHARDS': (3.0, 0.0),
    'ELL': (-1.5, 2.3),
    'CHURCHILL': (5.40, 5.5),
    'SNELL_LIB': (3.0, 6.5),      # wall at y=7.4, need clearance
    'SNELL_ENG': (4.0, 8.5),      # wall at x=4.5, shifted west
    'ORIGIN': (0.0, 0.0)
}

FALLBACK_ENTRANCES = {
    'ORIGIN': (0.0, 0.0),
    'DODGE': (-2.5, 0.0),
    'ELL': (-1.5, 2.3),
    'RICHARDS': (3.0, 0.0),
    'CURRY': (-1.5, 6.0),
    'SNELL_LIB': (3.0, 6.5),
    'CHURCHILL': (5.40, 5.5),
    'SNELL_ENG': (4.0, 8.5)
}


def strip_xml_namespace(tag: str) -> str:
    return tag.split('}', 1)[1] if '}' in tag else tag


def parse_world_file(path: str, prefix: str) -> list[Waypoint]:
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    tree = ET.parse(path)
    points = []

    for elem in tree.getroot().iter():
        if strip_xml_namespace(elem.tag) != "model":
            continue

        name = elem.get("name", "")
        if not name.startswith(prefix):
            continue

        pose = next((c for c in elem if strip_xml_namespace(c.tag) == "pose"), None)
        if pose is None or not pose.text:
            continue

        readable = MARKER_NAMES.get(name, name)
        
        if readable in SAFE_POSITIONS:
            x, y = SAFE_POSITIONS[readable]
            print(f"[tuning] {readable} -> ({x}, {y})")
        else:
            parts = pose.text.strip().split()
            x, y = float(parts[0]), float(parts[1])

        points.append((readable, x, y))

    return points


def distance(p1: Waypoint, p2: Waypoint) -> float:
    return math.hypot(p1[1] - p2[1], p1[2] - p2[2])


def build_distance_matrix(points: list[Waypoint]) -> list[list[float]]:
    n = len(points)
    return [[distance(points[i], points[j]) for j in range(n)] for i in range(n)]


def tour_length(tour: list[int], points: list[Waypoint]) -> float:
    return sum(distance(points[tour[i]], points[tour[i + 1]]) for i in range(len(tour) - 1))


def solve_christofides(points: list[Waypoint]) -> list[int]:
    """
    Standard Christofides: MST -> odd-vertex matching -> Eulerian circuit -> Hamiltonian path.
    Returns tour as list of indices (closed loop, first == last).
    """
    if not points:
        return []

    n = len(points)
    dist = build_distance_matrix(points)

    # Prim's MST
    key = [float("inf")] * n
    parent: list[Optional[int]] = [None] * n
    done = [False] * n
    key[0] = 0.0

    for _ in range(n):
        u = min((v for v in range(n) if not done[v]), key=lambda x: key[x], default=None)
        if u is None:
            break
        done[u] = True
        for v in range(n):
            if not done[v] and dist[u][v] < key[v]:
                key[v] = dist[u][v]
                parent[v] = u

    # Build adjacency from MST
    adj = [[] for _ in range(n)]
    for v, u in enumerate(parent):
        if u is not None:
            adj[u].append(v)
            adj[v].append(u)

    # Greedy matching on odd-degree vertices
    odd_vertices = {i for i, neighbors in enumerate(adj) if len(neighbors) % 2 == 1}
    while odd_vertices:
        u = odd_vertices.pop()
        v = min(odd_vertices, key=lambda x: dist[u][x])
        odd_vertices.remove(v)
        adj[u].append(v)
        adj[v].append(u)

    # Hierholzer's algorithm for Eulerian path
    stack = [0]
    path = []
    local_adj = [nbrs[:] for nbrs in adj]

    while stack:
        v = stack[-1]
        if local_adj[v]:
            u = local_adj[v].pop()
            local_adj[u].remove(v)
            stack.append(u)
        else:
            path.append(stack.pop())

    # Shortcut to Hamiltonian
    visited = set()
    tour = []
    for v in reversed(path):
        if v not in visited:
            visited.add(v)
            tour.append(v)

    tour.append(tour[0])
    return tour


class TourExecutor(Node):
    def __init__(self, waypoints: list[Waypoint], tour: list[int], total_dist: float):
        super().__init__('tour_executor')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = waypoints
        self.tour = tour
        self.total_dist = total_dist
        self.idx = 0
        self.start_time: Optional[float] = None

    def start(self):
        self.get_logger().info("Waiting for Nav2...")
        self._client.wait_for_server()
        self.get_logger().info(f"Starting tour ({self.total_dist:.2f}m planned)")
        self.start_time = time.time()
        self._send_next_goal()

    def _send_next_goal(self):
        if self.idx >= len(self.tour):
            self._finish()
            return

        name, x, y = self.waypoints[self.tour[self.idx]]
        self.get_logger().info(f"[{self.idx + 1}/{len(self.tour)}] {name} ({x:.2f}, {y:.2f})")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected - check for collisions")
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self.get_logger().info("Reached.")
        self.idx += 1
        time.sleep(1.0)
        self._send_next_goal()

    def _finish(self):
        elapsed = time.time() - self.start_time
        mins, secs = int(elapsed // 60), int(elapsed % 60)

        self.get_logger().info("=" * 35)
        self.get_logger().info("TOUR COMPLETE")
        self.get_logger().info(f"Distance: {self.total_dist:.2f}m | Time: {mins}m {secs}s")
        self.get_logger().info("=" * 35)

        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default=DEFAULT_WORLD_FILE)
    parser.add_argument("--prefix", default=DEFAULT_PREFIX)
    opts, ros_args = parser.parse_known_args()

    try:
        points = parse_world_file(opts.world, opts.prefix)
        print(f"Loaded {len(points)} waypoints")
    except Exception as e:
        print(f"[warn] Parse failed ({e}), using fallbacks")
        points = [(k, *v) for k, v in FALLBACK_ENTRANCES.items()]

    print("Solving TSP...")
    tour = solve_christofides(points)
    total_dist = tour_length(tour, points)

    print("\nRoute:")
    for i, idx in enumerate(tour):
        name, x, y = points[idx]
        print(f"  {i + 1}. {name:12s} ({x:.2f}, {y:.2f})")
    print(f"\nTotal: {total_dist:.2f}m\n")

    rclpy.init(args=ros_args)
    executor = TourExecutor(points, tour, total_dist)
    executor.start()

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
