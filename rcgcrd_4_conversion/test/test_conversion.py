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

from rcgcrd_4.msg import RCGCRD

from rcgcrd_4_conversion.conversion import data_to_msg, msg_to_data
from rcgcrd_4_conversion.robocup_game_control_return_data import RoboCupGameControlReturnData


def test_msg_to_data():
    """Convert RCGCRD msg to binary data."""
    msg = RCGCRD()
    msg.player_num = 1
    msg.team_num = 2
    msg.fallen = 1
    msg.pose = [1.0, 2.0, 3.0]
    msg.ball_age = 20.0
    msg.ball = [10.0, 20.0]
    data = msg_to_data(msg)

    parsed = RoboCupGameControlReturnData.parse(data)
    assert parsed.playerNum == 1
    assert parsed.teamNum == 2
    assert parsed.fallen == 1
    assert parsed.pose == [1.0, 2.0, 3.0]
    assert parsed.ballAge == 20.0
    assert parsed.ball == [10.0, 20.0]


def test_data_to_msg():
    """Convert binary data to RCGCRD msg."""
    container = Container(
        playerNum=1,
        teamNum=2,
        fallen=1,
        pose=[1.0, 2.0, 3.0],
        ballAge=20.0,
        ball=[10.0, 20.0]
    )
    data = RoboCupGameControlReturnData.build(container)
    msg = data_to_msg(data)
    assert msg.player_num == 1
    assert msg.team_num == 2
    assert msg.fallen == 1
    assert msg.pose[0] == 1.0
    assert msg.pose[1] == 2.0
    assert msg.pose[2] == 3.0
    assert msg.ball_age == 20.0
    assert msg.ball[0] == 10.0
    assert msg.ball[1] == 20.0
