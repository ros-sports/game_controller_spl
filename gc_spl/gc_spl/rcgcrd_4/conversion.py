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

from construct import Container

from gc_spl.rcgcrd_4.robocup_game_control_return_data import \
    RoboCupGameControlReturnData
from gc_spl_interfaces.msg import RCGCRD4


def rcgcrd_msg_to_data(msg: RCGCRD4) -> bytes:
    """Convert RCGCRD ROS msg to binary data."""
    container = Container(
        playerNum=msg.player_num,
        teamNum=msg.team_num,
        fallen=msg.fallen,
        pose=msg.pose,
        ballAge=msg.ball_age,
        ball=msg.ball
    )
    data = RoboCupGameControlReturnData.build(container)
    return data
