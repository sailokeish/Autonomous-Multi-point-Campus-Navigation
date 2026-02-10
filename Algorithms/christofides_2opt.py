#!/usr/bin/env python3
"""Christofides + 2-opt optimization for robot campus tour."""

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

# Tuned to stay clear of walls while remaining reachable
SAFE_POSITIONS = {
    'RICHARDS': (3.0, 0.0),       # wall at y=0.28
    'ELL': (-1.5, 2.3),           # wall at y=2.83
    'CHURCHILL': (5.40, 5.5),     # wall at x=5.66
    'ORIGIN': (0.0, 0.0)
}

FALLBACK_ENTRANCES = {
    'ORIGIN': (0.0, 0.0),
    'DODGE': (-2.5, 0.0),
    'ELL': (-1.5, 2.3),
    'RICHARDS': (3.0, 0.0),
    'CURRY': (-1.5, 6.0),
    'SNELL_LIB': (3.0, 7.35),
    'CHURCHILL': (5.40, 5.5),
    'SNELL_ENG': (4.5, 8.5)
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


def solve_tsp(points: list[Waypoint]) -> list[int]:
    """Christofides base tour + 2-opt local search."""
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
    odd_vertices = {i for i, nbrs in enumerate(adj) if len(nbrs) % 2 == 1}
    while odd_vertices:
        u = odd_vertices.pop()
        v = min(odd_vertices, key=lambda x: dist[u][x])
        odd_vertices.remove(v)
        adj[u].append(v)
        adj[v].append(u)

    # Hierholzer's for Eulerian path
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

    # 2-opt improvement
    route = tour[:]
    size = len(route)
    improved = True

    while improved:
        improved = False
        for i in range(size - 1):
            for j in range(i + 2, size):
                a, b = route[i], route[(i + 1) % size]
                c, d = route[j], route[(j + 1) % size]

                # swap if it shortens the path
                if dist[a][c] + dist[b][d] < dist[a][b] + dist[c][d]:
                    route[i + 1:j + 1] = reversed(route[i + 1:j + 1])
                    improved = True

    route.append(route[0])
    return route


class TourExecutor(Node):
    def __init__(self, waypoints: list[Waypoint], tour: list[int]):
        super().__init__('tour_executor_2opt')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = waypoints
        self.tour = tour
        self.idx = 0

    def start(self):
        self.get_logger().info("Waiting for Nav2...")
        self._client.wait_for_server()
        self.get_logger().info("Starting optimized tour")
        self._send_next_goal()

    def _send_next_goal(self):
        if self.idx >= len(self.tour):
            self.get_logger().info("TOUR COMPLETE")
            rclpy.shutdown()
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
            self.get_logger().error("Goal rejected - might be in inflation layer")
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self.get_logger().info("Reached.")
        self.idx += 1
        time.sleep(1.0)
        self._send_next_goal()


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

    print("Solving TSP (Christofides + 2-opt)...")
    tour = solve_tsp(points)

    print("\nOptimized route:")
    for i, idx in enumerate(tour):
        name, x, y = points[idx]
        print(f"  {i + 1}. {name:12s} ({x:.2f}, {y:.2f})")
    print()

    rclpy.init(args=ros_args)
    executor = TourExecutor(points, tour)
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
