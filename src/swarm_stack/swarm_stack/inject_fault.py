#!/usr/bin/env python3
"""Inject a reproducible UAV, link, or emergency-PoI disturbance."""

import argparse
import time

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import FaultEvent


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--event_type", required=True,
                        choices=["uav_failure", "link_degraded", "link_restored", "new_poi"])
    parser.add_argument("--target", required=True,
                        help="UAV name, edge such as GCS:drone2, or new PoI ID")
    parser.add_argument("--description", default="")
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--priority", type=int, default=10)
    parsed = parser.parse_args(args)
    if parsed.event_type == "new_poi" and (parsed.x is None or parsed.y is None):
        parser.error("new_poi requires --x and --y world-frame coordinates")
    rclpy.init()
    node = Node("inject_fault_client")
    publisher = node.create_publisher(FaultEvent, "/swarm/fault_events", 10)
    start = time.monotonic()
    # Mission manager, communication model, and metrics logger must all be
    # matched before the one-shot event is sent.
    while publisher.get_subscription_count() < 3 and time.monotonic() - start < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() < 3:
        node.get_logger().error("Fault event not sent: fewer than three swarm subscribers are connected")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    event = FaultEvent()
    event.event_type, event.target, event.description = parsed.event_type, parsed.target, parsed.description
    event.x, event.y, event.z, event.priority = parsed.x or 0.0, parsed.y or 0.0, parsed.z, parsed.priority
    publisher.publish(event)
    node.get_logger().info(f"Published {parsed.event_type}: {parsed.target}")
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
