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

from rcgcrd_spl_4.msg import RCGCRD

from rcgcrd_spl_4_conversion.conversion import rcgcrd_msg_to_data
from rcgcrd_spl_4_conversion.robocup_game_control_return_data import RoboCupGameControlReturnData


def test_rcgcrd_msg_to_data():
    """Test conversion of RCGCRD msg to binary data."""
    msg = RCGCRD()
    msg.player_num = 1
    msg.team_num = 2
    msg.fallen = 1
    msg.pose = [1.0, 2.0, 3.0]
    msg.ball_age = 20.0
    msg.ball = [10.0, 20.0]
    data = rcgcrd_msg_to_data(msg)

    parsed = RoboCupGameControlReturnData.parse(data)
    assert parsed.playerNum == 1
    assert parsed.teamNum == 2
    assert parsed.fallen == 1
    assert parsed.pose == [1.0, 2.0, 3.0]
    assert parsed.ballAge == 20.0
    assert parsed.ball == [10.0, 20.0]
