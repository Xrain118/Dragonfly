#!/usr/bin/env python3
"""Capture the running Gazebo model using temporary scene cameras.

Avoids WSLg screenshot limitations. Cameras have no collision/inertia and
are deleted after each capture. This script does not move robot joints.
Run after gazebo.launch.py (unpaused). Requires cv_bridge and OpenCV.
"""
import argparse
import math
from pathlib import Path
import time
import uuid

import cv2
from cv_bridge import CvBridge
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def request(node, client, value):
    if not client.wait_for_service(timeout_sec=10):
        raise RuntimeError("Gazebo entity service unavailable")
    future = client.call_async(value)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    if not future.done() or future.result() is None:
        raise RuntimeError("Gazebo entity service timed out")
    if not future.result().success:
        raise RuntimeError(future.result().status_message)


def capture(node, output, label, eye, target, fov):
    name = "validation_camera_" + uuid.uuid4().hex[:10]
    dx, dy, dz = [b-a for a, b in zip(eye, target)]
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    yaw = math.atan2(dy, dx)
    xml = f'''<sdf version="1.6"><model name="{name}">
      <static>true</static><pose>{eye[0]} {eye[1]} {eye[2]} 0 {pitch} {yaw}</pose>
      <link name="camera_link"><sensor name="camera" type="camera">
        <always_on>true</always_on><update_rate>2</update_rate>
        <camera><horizontal_fov>{fov}</horizontal_fov>
          <image><width>1440</width><height>1080</height><format>R8G8B8</format></image>
          <clip><near>0.01</near><far>100</far></clip>
        </camera>
        <plugin name="camera_ros" filename="libgazebo_ros_camera.so">
          <ros><namespace>/{name}</namespace></ros>
          <camera_name>{name}</camera_name><frame_name>camera_link</frame_name>
        </plugin>
      </sensor></link></model></sdf>'''
    frames = []
    sub = node.create_subscription(Image, f"/{name}/{name}/image_raw", frames.append, qos_profile_sensor_data)
    spawn = node.create_client(SpawnEntity, "/spawn_entity")
    delete = node.create_client(DeleteEntity, "/delete_entity")
    spawned = False
    try:
        req = SpawnEntity.Request(name=name, xml=xml)
        request(node, spawn, req)
        spawned = True
        deadline = time.monotonic() + 30
        while len(frames) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if not frames:
            raise RuntimeError(f"No camera frame for {label}")
        pixels = CvBridge().imgmsg_to_cv2(frames[-1], "bgr8")
        if float(pixels.std()) < 2:
            raise RuntimeError(f"Camera {label} returned a blank frame")
        path = output / f"{label}.png"
        if not cv2.imwrite(str(path), pixels):
            raise RuntimeError(f"Failed to save {path}")
        print(path, flush=True)
    finally:
        if spawned:
            request(node, delete, DeleteEntity.Request(name=name))
        node.destroy_subscription(sub)
        node.destroy_client(spawn)
        node.destroy_client(delete)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("capture_model_validation")
    try:
        capture(node, args.output, "isometric", (1.7, -2.0, 2.65), (0, 0, 1.42), 0.82)
        capture(node, args.output, "side", (0.0, -2.4, 1.42), (0, 0, 1.42), 0.82)
        capture(node, args.output, "arm_detail", (0.45, -0.65, 1.37), (0, 0, 1.36), 0.72)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
