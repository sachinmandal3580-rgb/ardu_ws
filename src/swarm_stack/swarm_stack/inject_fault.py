#!/usr/bin/env python3
"""
inject_fault
------------
One-shot CLI to publish a FaultEvent, for exercising the swarm's fault
recovery during a demonstration or test.

Usage:
  ros2 run swarm_stack inject_fault --event_type uav_failure --target drone3
  ros2 run swarm_stack inject_fault --event_type new_poi --target poi_6 \\
      --description "New high-priority PoI reported by GCS"
"""

import argparse
import time

import rclpy
from rclpy.node import Node

from swarm_interfaces.msg import FaultEvent


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--event_type", required=True,
                         choices=["uav_failure", "link_degraded", "new_poi"])
    parser.add_argument("--target", required=True)
    parser.add_argument("--description", default="")
    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = Node("inject_fault_client")
    pub = node.create_publisher(FaultEvent, "/swarm/fault_events", 10)
    time.sleep(0.5)  # allow discovery before publishing, since this is one-shot

    msg = FaultEvent()
    msg.event_type = parsed.event_type
    msg.target = parsed.target
    msg.description = parsed.description
    pub.publish(msg)
    node.get_logger().info(f"Published fault event: {parsed.event_type} -> {parsed.target}")

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
