#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Vive Tracker module - get device pose data and visualize it in real time using Pygame
This example is to help users better observe if the device positioning has drift
"""

import sys  # noqa: F401
import time
import os  # noqa: F401
import logging
import numpy as np
import pygame
import math  # noqa: F401

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
        :param position: 坐标系原点 [x, y, z]
        :param quaternion: 坐标系姿态 [x, y, z, w]
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
        quat_text = f"Rotation (x, y, z, w): ({tracker_rotation[0]:.4f}, {tracker_rotation[1]:.4f}, {tracker_rotation[2]:.4f}, {tracker_rotation[3]:.4f})"
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


# --- Vive Tracker logic ---
try:
    from pika.sense import Sense

    def run_visualization():
        """test getting pose data of WM0 device and visualize it"""
        viz = PygameVisualizer()
        if not viz.is_running():
            logger.error("Pygame visualization environment not ready, exiting.")
            return False

        sense = Sense()
        logger.info("connecting Sense device...")
        if not sense.connect():
            logger.error("connecting Sense device failed")
            pygame.quit()
            return False

        try:
            tracker = sense.get_vive_tracker()
            if not tracker:
                logger.error("getting Vive Tracker object failed, please ensure pysurvive library is installed")
                pygame.quit()
                return False

            logger.info("waiting for device initialization to complete...")
            time.sleep(2.0)

            target_device = "WM0"
            max_retries = 10
            retry_count = 0
            # device connection check logic...
            for retry_count in range(max_retries):
                devices = sense.get_tracker_devices()
                if target_device in devices:
                    logger.info(f"successfully detected {target_device} device!")
                    break
                else:
                    logger.info(
                        f"not detected {target_device} device, waiting and retrying ({retry_count+1}/{max_retries})..."
                    )
                time.sleep(1.0)
            else:
                logger.warning(f"after multiple attempts, still not detected {target_device} device")
                pygame.quit()
                return False

            logger.info(
                f"start getting pose data of {target_device} device and updating visualization (W/S or Up/Down keys to control zoom, press X to close window)..."
            )

            # loop to get data and update visualization
            while viz.is_running():
                pose = sense.get_pose(target_device)

                if pose:
                    position = pose.position  # [x, y, z]
                    rotation = pose.rotation  # [x, y, z， w] quaternion
                    viz.update(position, rotation)
                else:
                    logger.warning(f"cannot get pose data of {target_device}...")
                    # at least call update once to handle user input events
                    viz.update([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

                time.sleep(0.01)

        except Exception as e:
            logger.error(f"error occurred during getting pose data: {e}")
        finally:
            logger.info("disconnecting Vive Tracker device...")
            sense.disconnect()
            pygame.quit()

    if __name__ == "__main__":
        run_visualization()

except ImportError as e:
    logger.error(f"import error occurred: {e}")
    logger.error("missing dependencies, ensure pysurvive library, pika.sense library and pygame library are installed")
