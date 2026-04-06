#!/usr/bin/env python

# Copyright 2026 APPEAL Automation team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

"""Processor step that maps PiPER normalised actions to Fairino degrees.

PiPER leader outputs normalised joint values in -100..100.
Fairino follower expects absolute joint angles in degrees.
This step linearly maps each joint from the normalised range
onto the Fairino joint limits so that 0 maps to the midpoint.
"""

from lerobot.configs.types import PipelineFeatureType, PolicyFeature
from lerobot.types import EnvTransition, TransitionKey

from .pipeline import ProcessorStep, ProcessorStepRegistry

# Fairino FR5 joint limits [deg] per joint index.
_FR5_LOWER = {
    "joint1": -175.0,
    "joint2": -265.0,
    "joint3": -160.0,
    "joint4": -265.0,
    "joint5": -175.0,
    "joint6": -175.0,
}
_FR5_UPPER = {
    "joint1": 175.0,
    "joint2": 85.0,
    "joint3": 160.0,
    "joint4": 85.0,
    "joint5": 175.0,
    "joint6": 175.0,
}

# PiPER normalised range for joints.
_PIPER_NORM_MIN = -100.0
_PIPER_NORM_MAX = 100.0


@ProcessorStepRegistry.register("PiperToFairinoStep")
class PiperToFairinoStep(ProcessorStep):
    """Map PiPER normalised joint values to Fairino degrees.

    For each ``joint{1..6}.pos`` key in the action dict the
    normalised value (-100..100) is linearly mapped to the
    corresponding Fairino joint range.  Keys that do not match
    (e.g. ``gripper.pos``) are dropped because Fairino has no
    gripper.
    """

    def __call__(
        self, transition: EnvTransition,
    ) -> EnvTransition:
        self._current_transition = transition
        action = transition.get(TransitionKey.ACTION)
        if action is None:
            return transition

        mapped = {}
        for jname in _FR5_LOWER:
            key = f"{jname}.pos"
            norm = action.get(key, 0.0)

            # Clamp to normalised range.
            norm = max(
                _PIPER_NORM_MIN,
                min(_PIPER_NORM_MAX, float(norm)),
            )

            lo = _FR5_LOWER[jname]
            hi = _FR5_UPPER[jname]
            mid = (lo + hi) / 2.0
            half = (hi - lo) / 2.0

            mapped[key] = (
                mid
                + (norm / _PIPER_NORM_MAX) * half
            )

        transition[TransitionKey.ACTION] = mapped
        return transition

    def transform_features(
        self,
        features: dict[
            PipelineFeatureType, dict[str, PolicyFeature]
        ],
    ) -> dict[
        PipelineFeatureType, dict[str, PolicyFeature]
    ]:
        return features
