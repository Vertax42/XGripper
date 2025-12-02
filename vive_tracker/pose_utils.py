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
import numpy as np


# helper function: turn quaternion and position to 4x4 transformation matrix
def xyzQuaternion2matrix(x, y, z, qx, qy, qz, qw):
    """
    turn quaternion and position to 4x4 transformation matrix
    """
    # create rotation matrix
    rot_matrix = np.array(
        [
            [
                1 - 2 * qy * qy - 2 * qz * qz,
                2 * qx * qy - 2 * qz * qw,
                2 * qx * qz + 2 * qy * qw,
                0,
            ],
            [
                2 * qx * qy + 2 * qz * qw,
                1 - 2 * qx * qx - 2 * qz * qz,
                2 * qy * qz - 2 * qx * qw,
                0,
            ],
            [
                2 * qx * qz - 2 * qy * qw,
                2 * qy * qz + 2 * qx * qw,
                1 - 2 * qx * qx - 2 * qy * qy,
                0,
            ],
            [0, 0, 0, 1],
        ]
    )

    # set translation part
    rot_matrix[0, 3] = x
    rot_matrix[1, 3] = y
    rot_matrix[2, 3] = z

    return rot_matrix


# helper function: turn xyz position and rpy angle to 4x4 transformation matrix
def xyzrpy2Mat(x, y, z, roll, pitch, yaw):
    """
    turn xyz position and rpy angle to 4x4 transformation matrix
    """
    # create rotation matrix
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rot_matrix = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
            [-sp, cp * sr, cp * cr, z],
            [0, 0, 0, 1],
        ]
    )

    return rot_matrix


# helper function: turn 4x4 transformation matrix to position and quaternion
def matrixToXYZQuaternion(matrix):
    """
    turn 4x4 transformation matrix to position and quaternion
    """
    # extract position
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]

    # extract rotation matrix
    rot_matrix = matrix[:3, :3]

    # calculate quaternion
    trace = rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) * s
        qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) * s
        qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) * s
    elif rot_matrix[0, 0] > rot_matrix[1, 1] and rot_matrix[0, 0] > rot_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[0, 0] - rot_matrix[1, 1] - rot_matrix[2, 2])
        qw = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
        qz = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
    elif rot_matrix[1, 1] > rot_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[1, 1] - rot_matrix[0, 0] - rot_matrix[2, 2])
        qw = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
        qx = (rot_matrix[0, 1] + rot_matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot_matrix[2, 2] - rot_matrix[0, 0] - rot_matrix[1, 1])
        qw = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
        qx = (rot_matrix[0, 2] + rot_matrix[2, 0]) / s
        qy = (rot_matrix[1, 2] + rot_matrix[2, 1]) / s
        qz = 0.25 * s

    return x, y, z, qx, qy, qz, qw
