#!/usr/bin/env python

# Copyright APPEAL Automation team. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.cameras import Camera
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.constants import HF_LEROBOT_CALIBRATION, ROBOTS
from lerobot.errors import DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.robots import Robot
from lerobot.robots.fairino_follower.config_fairino_follower import FairinoFollowerConfig
from lerobot.robots.utils import ensure_safe_goal_position

from fairino import Robot

logger = logging.getLogger(__name__)

class FairinoFollower(Robot):
    '''
    Declare class for later usage
    '''
    config_class: FairinoFollowerConfig
    name = "fairino_follower"
    cameras : dict[str, Camera] = {}

    def __init__(self, config: FairinoFollowerConfig):
        '''
        Link fairinoSDK to lerobot architecture
        '''
        super().__init__(config)
        self.config = config
        self._robot: Robot | None = None
        self._is_connected = False
    
    def connect(self) -> None:
        '''
        Connect fairino robot
        '''
        print(f"[Fairino] Connecting to {self.config.robot_ip}")
        self._robot = FaiRobot.RPC(self.config.robot_ip)

        # Check joint position
        ret, joints = self._robot.GetActualJointPosDegree()
        if ret != 0:
            raise ConnectionError(
                f"[Fairino] Fail to connect / (Error code: {ret})\n"
            )

        self._is_connected = True
        print(f"[Fairino] Success to connect / (Joint_status: {joints})\n")
        
        # Check camera
        for cam_name, cam_config in self.config.cameras.items():
            cam_config.connect()
            print(f"[Fairino] Success to connect / (Camera: {cam_name})\n")
    
    def disconnect(self) -> None:
        '''
        Disconnect fairino robot
        '''
        if self._robot is not None:
            self._robot = None
            self._is_connected = False

        print("[Fairino] Success to disconnect")
    
    def get_observation(self) -> dict[str, Any]:
        """
        Return fairino robot's joint and camera oberservation data

        Ouput: dict with keys
            "observation.state": (radian, shape: [6]), 
            "observation.images.<cam_name>": camera_image array)
        """
        self._check_connected()

        obs = {}

        # joint status (degree)
        ret, joints_deg = self._robot.GetActualJointPosDegree()
        if ret != 0:
            raise RuntimeError(f"[Fairino] Fail to read joint / (error code: {ret})")

        joints = np.array(joints_deg, dtype=np.float32)

        # degree2radian (for LeRobot standard)
        if self.config.use_radian:
            joints = np.deg2rad(joints)

        obs["observation.state"] = joints

        # camera_image
        for cam_name, cam in self.config.cameras.items():
            img = cam.read()
            obs[f"observation.images.{cam_name}"] = img

        return obs
    
    def send_action(self, action: np.ndarray) -> np.ndarray:
        """
        Move fairino robot's joint to target location

        Input: Target joint location (radian, shape: [6])
        Output: Real joint location
        """
        self._check_connected()

        # radian2degree (for Fairino standard)
        if self.config.use_radian:
            action_deg = np.rad2deg(action)
        else:
            action_deg = action.copy()

        # Input filter
        lower = np.array(self.config.joint_limits_lower)
        upper = np.array(self.config.joint_limits_upper)
        action_deg = np.clip(action_deg, lower, upper)

        # Fairino SDK: MoveJ (관절 공간 이동)
        # 파라미터: joint_pos(list), speed(%), acc(%), tol, ovl, blendT
        joint_list = action_deg.tolist()
        speed = self.config.move_speed * 100  # 0~100%

        ret = self._robot.MoveJ(
            joint_list,   # Target joint
            speed,        # Speed (%)
            speed,        # Velocity (%)
            0,            # error_margin
            100,          # override (%)
            -1,           # disable blending (%) 
            tool = 1,     # robot_specific_parameter
            user = 0      # robot_specific_parameter  
        )

        if ret != 0:
            print(f"[Fairino] Fail to move joint / (error code: {ret})")

        return np.deg2rad(action_deg) if self.config.use_radian else action_deg

