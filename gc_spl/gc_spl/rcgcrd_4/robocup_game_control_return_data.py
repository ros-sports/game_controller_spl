# Copyright 2022 Kenji Brameld
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

from construct import Array, Byte, Const, Default, Float32l, Struct

GAMECONTROLLER_RETURN_PORT = 3939

GAMECONTROLLER_RETURN_STRUCT_HEADER = b'RGrt'
GAMECONTROLLER_RETURN_STRUCT_VERSION = 4

RoboCupGameControlReturnData = Struct(
    'header' / Const(GAMECONTROLLER_RETURN_STRUCT_HEADER),  # "RGrt"
    'version' / Const(GAMECONTROLLER_RETURN_STRUCT_VERSION, Byte),  # has to be set to GAMECONTROLLER_RETURN_STRUCT_VERSION  # noqa: E501
    'playerNum' / Default(Byte, 0),  # player number starts with 1
    'teamNum' / Default(Byte, 0),  # team number
    'fallen' / Default(Byte, 255),  # 1 means that the robot is fallen, 0 means that the robot can play  # noqa: E501
    # position and orientation of robot
    # coordinates in millimeters
    # 0,0 is in center of field
    # +ve x-axis points towards the goal we are attempting to score on
    # +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
    # angle in radians, 0 along the +x axis, increasing counter clockwise
    'pose' / Default(Array(3, Float32l), [0, 0, 0]),  # x,y,theta
    # ball information
    # seconds since this robot last saw the ball. -1.f if we haven't seen it
    'ballAge' / Default(Float32l, -1),
    # position of ball relative to the robot
    # coordinates in millimeters
    # 0,0 is in center of the robot
    # +ve x-axis points forward from the robot
    # +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
    'ball' / Default(Array(2, Float32l), [0, 0])
)
