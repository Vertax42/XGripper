#!/usr/bin/env python

# Copyright 2025 The Xense Robotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Vive Tracker module - get device pose data and visualize it in real time using Pygame
This example is to help users better observe if the device positioning has drift
"""

import sys  # noqa: F401
from pathlib import Path  # noqa: F401

import time
import logging
import argparse
import numpy as np
import pygame

from xensesdk.ezgl.utils.QtTools import qtcv  # noqa: F401
from xesne_gripper import FlareGrip

# configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("vive_tracker_visualization")


# --- Pygame visualization class ---
class PygameVisualizer:
    def __init__(self, width=800, height=600):
        try:
            pygame.init()
            self.screen = pygame.display.set_mode((width, height))
            pygame.display.set_caption("Vive Tracker Pose Visualization")
            self.clock = pygame.time.Clock()
            self.font = pygame.font.Font(None, 36)
            self.running = True
        except Exception as e:
            logger.error(f"Pygame initialization failed: {e}")
            self.running = False

        # 1. add zoom/camera parameters
        self.camera_distance = 5.0  # distance from z=0 plane
        self.scale_factor = 2000.0  # projection scale factor, for scaling the view
        self.zoom_step = 100.0  # adjustment amount for each zoom

    def project_3d_to_2d(self, point_3d):
        """project 3D point to 2D screen"""
        x, y, z = point_3d

        # avoid division by zero or negative value
        depth = z + self.camera_distance
        if depth <= 0.1:
            depth = 0.1

        # use scale factor to control view size
        factor = self.scale_factor / depth
        x_2d = 400 + x * factor
        y_2d = 300 - y * factor
        return (int(x_2d), int(y_2d))

    def quaternion_to_rotation_matrix(self, quaternion):
        """convert [x, y, z, w] quaternion to 3x3 rotation matrix"""
        q1, q2, q3, q0 = quaternion  # corresponding to (x, y, z, w)

        R = np.array(
            [
                [
                    1 - 2 * (q2**2 + q3**2),
                    2 * (q1 * q2 - q0 * q3),
                    2 * (q1 * q3 + q0 * q2),
                ],
                [
                    2 * (q1 * q2 + q0 * q3),
                    1 - 2 * (q1**2 + q3**2),
                    2 * (q2 * q3 - q0 * q1),
                ],
                [
                    2 * (q1 * q3 - q0 * q2),
                    2 * (q2 * q3 + q0 * q1),
                    1 - 2 * (q1**2 + q2**2),
                ],
            ]
        )
        return R

    def draw_coordinate_frame(self, position, quaternion, color, length, name=""):
        """
        draw a coordinate frame
        :param position: coordinate frame origin [x, y, z]
        :param quaternion: coordinate frame orientation [x, y, z, w]
        :param color: axis color (R, G, B)
        :param length: axis length
        :param name: coordinate frame name, for display
        """
        # convert quaternion to rotation matrix
        R = self.quaternion_to_rotation_matrix(quaternion)

        # coordinate frame axes
        pos_np = np.array(position)
        x_end = pos_np + R[:, 0] * length
        y_end = pos_np + R[:, 1] * length
        z_end = pos_np + R[:, 2] * length

        # project to 2D
        pos_2d = self.project_3d_to_2d(pos_np)
        x_end_2d = self.project_3d_to_2d(x_end)
        y_end_2d = self.project_3d_to_2d(y_end)
        z_end_2d = self.project_3d_to_2d(z_end)

        # draw axis lines (using the incoming color)
        pygame.draw.line(self.screen, (255, 0, 0), pos_2d, x_end_2d, 3)  # X axis - red
        pygame.draw.line(self.screen, (0, 255, 0), pos_2d, y_end_2d, 3)  # Y axis - green
        pygame.draw.line(self.screen, (0, 0, 255), pos_2d, z_end_2d, 3)  # Z axis - blue

        # draw origin
        pygame.draw.circle(self.screen, color, pos_2d, 5)

        # show coordinate frame name
        if name:
            text_surface = self.font.render(name, True, color)
            self.screen.blit(text_surface, (pos_2d[0] + 10, pos_2d[1] - 10))

    def handle_input(self):
        """handle user input, including exit and zoom"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return

            if event.type == pygame.KEYDOWN:
                # zoom in (W key or Up Arrow)
                if event.key == pygame.K_w or event.key == pygame.K_UP:
                    self.scale_factor += self.zoom_step
                    logger.info(f"zoom: zoom in to {self.scale_factor:.2f}")
                # zoom out (S key or Down Arrow)
                if event.key == pygame.K_s or event.key == pygame.K_DOWN:
                    if self.scale_factor > self.zoom_step:
                        self.scale_factor -= self.zoom_step
                        logger.info(f"zoom: zoom out to {self.scale_factor:.2f}")

    def update(self, tracker_position, tracker_rotation):
        """update display, draw base link and tracker coordinate frame"""
        self.handle_input()
        if not self.running:
            return

        self.screen.fill((0, 0, 0))  # black background

        # --- 1. draw base link coordinate frame (origin) ---
        # origin position, no rotation
        base_pos = [0.0, 0.0, 0.0]
        base_quat = [0.0, 0.0, 0.0, 1.0]
        self.draw_coordinate_frame(
            position=base_pos,
            quaternion=base_quat,
            color=(100, 100, 255),  # light blue
            length=0.7,  # axis length 0.7m
            name="base_Link",
        )

        # --- 2. draw tracker coordinate frame (relative to base link) ---
        # tracker_position and tracker_rotation are the pose of the tracker relative to the base link
        # (world coordinate system) in local coordinate system, which is the coordinate system of the base link
        self.draw_coordinate_frame(
            position=tracker_position,
            quaternion=tracker_rotation,
            color=(255, 255, 0),  # yellow
            length=0.5,  # axis length 0.5m
            name="tracker_link",
        )

        # draw line from base link to tracker (to clearly show the relative position)
        pos_2d_base = self.project_3d_to_2d(base_pos)
        pos_2d_tracker = self.project_3d_to_2d(tracker_position)
        pygame.draw.line(self.screen, (150, 150, 150), pos_2d_base, pos_2d_tracker, 1)

        # show pose information
        info_text = f"Position: ({tracker_position[0]:.4f}, {tracker_position[1]:.4f}, {tracker_position[2]:.4f}) (m)"
        quat_text = (
            f"Rotation (x, y, z, w): ({tracker_rotation[0]:.4f}, {tracker_rotation[1]:.4f}, "
            f"{tracker_rotation[2]:.4f}, {tracker_rotation[3]:.4f})"
        )
        zoom_text = f"Zoom/Scale: {self.scale_factor/200.0:.2f}x (W/S or Up/Down)"

        text_surface = self.font.render(info_text, True, (255, 255, 255))
        self.screen.blit(text_surface, (10, 10))

        quat_surface = self.font.render(quat_text, True, (255, 255, 255))
        self.screen.blit(quat_surface, (10, 40))

        zoom_surface = self.font.render(zoom_text, True, (200, 200, 200))
        self.screen.blit(zoom_surface, (10, 70))

        pygame.display.flip()
        self.clock.tick(60)  # keep 60 FPS

    def is_running(self):
        return self.running


def run_visualization(mac_addr: str, target_device: str = None):
    """Run Vive Tracker visualization using FlareGrip"""
    viz = PygameVisualizer()
    if not viz.is_running():
        logger.error("Pygame visualization environment not ready, exiting.")
        return False

    logger.info(f"Initializing FlareGrip (MAC: {mac_addr})...")
    flare = FlareGrip(
        mac_addr=mac_addr,
        log_level=logging.INFO,
        no_gripper=True,
        no_sensor=True,
        no_cam=True,
    )

    try:
        tracker = flare.get_vive_tracker()
        if not tracker:
            logger.error("Failed to get Vive Tracker object, please ensure pysurvive is installed")
            pygame.quit()
            return False

        logger.info("Waiting for device initialization to complete...")
        time.sleep(2.0)

        # Get available devices and filter trackers
        all_devices = tracker.get_devices()
        trackers = [d for d in all_devices if not d.startswith("LH")]
        logger.info(f"Detected devices: {all_devices}")
        logger.info(f"Available trackers: {trackers}")

        # Determine target device
        if target_device:
            if target_device not in trackers:
                logger.warning(f"Specified device {target_device} not found in {trackers}")
                if trackers:
                    target_device = trackers[0]
                    logger.info(f"Using first available tracker: {target_device}")
                else:
                    logger.error("No trackers available")
                    pygame.quit()
                    return False
        else:
            if trackers:
                target_device = trackers[0]
                logger.info(f"Using first available tracker: {target_device}")
            else:
                # Wait and retry for tracker detection
                max_retries = 10
                for retry_count in range(max_retries):
                    all_devices = tracker.get_devices()
                    trackers = [d for d in all_devices if not d.startswith("LH")]
                    if trackers:
                        target_device = trackers[0]
                        logger.info(f"Detected tracker: {target_device}")
                        break
                    else:
                        logger.info(
                            f"No tracker detected, waiting and retrying ({retry_count+1}/{max_retries})..."
                        )
                    time.sleep(1.0)
                else:
                    logger.error("After multiple attempts, still no tracker detected")
                    pygame.quit()
                    return False

        logger.info(
            f"Starting pose visualization for {target_device} "
            "(W/S or Up/Down keys to zoom, close window to exit)..."
        )

        # Loop to get data and update visualization
        while viz.is_running():
            pose = tracker.get_pose(target_device)

            if pose:
                position = pose.position  # [x, y, z]
                rotation = pose.rotation  # [x, y, z, w] quaternion
                viz.update(position, rotation)
            else:
                # No pose data, update with default values
                viz.update([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

            time.sleep(0.01)

    except Exception as e:
        logger.error(f"Error occurred during pose visualization: {e}")
    finally:
        logger.info("Closing FlareGrip...")
        flare.close()
        pygame.quit()

    return True


def main():
    parser = argparse.ArgumentParser(description="Vive Tracker Visualization")
    parser.add_argument(
        "--mac",
        type=str,
        default="6ebbc5f53240",
        help="MAC address of the FlareGrip device",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Target Vive Tracker device name (e.g., WM0, T20)",
    )
    args = parser.parse_args()

    run_visualization(args.mac, args.device)


if __name__ == "__main__":
    main()
