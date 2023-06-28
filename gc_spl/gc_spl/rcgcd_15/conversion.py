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

from gc_spl.rcgcd_15.robocup_game_control_data import (
    MAX_NUM_PLAYERS, RoboCupGameControlData)
from gc_spl_interfaces.msg import RCGCD15


def rcgcd_data_to_msg(data: bytes) -> RCGCD15:
    """Convert binary data to RCGCRD ROS msg."""
    parsed = RoboCupGameControlData.parse(data)
    msg = RCGCD15()
    msg.packet_number = parsed.packetNumber
    msg.players_per_team = parsed.playersPerTeam
    msg.competition_phase = parsed.competitionPhase
    msg.competition_type = parsed.competitionType
    msg.game_phase = parsed.gamePhase
    msg.state = parsed.state
    msg.set_play = parsed.setPlay
    msg.first_half = parsed.firstHalf
    msg.kicking_team = parsed.kickingTeam
    msg.secs_remaining = parsed.secsRemaining
    msg.secondary_time = parsed.secondaryTime
    for t in range(2):
        msg.teams[t].team_number = parsed.teams[t].teamNumber
        msg.teams[t].field_player_colour = parsed.teams[t].fieldPlayerColour
        msg.teams[t].goalkeeper_colour = parsed.teams[t].goalkeeperColour
        msg.teams[t].goalkeeper = parsed.teams[t].goalkeeper
        msg.teams[t].score = parsed.teams[t].score
        msg.teams[t].penalty_shot = parsed.teams[t].penaltyShot
        msg.teams[t].single_shots = parsed.teams[t].singleShots
        msg.teams[t].message_budget = parsed.teams[t].messageBudget
        for p in range(MAX_NUM_PLAYERS):
            msg.teams[t].players[p].penalty = parsed.teams[t].players[p].penalty
            msg.teams[t].players[p].secs_till_unpenalised = \
                parsed.teams[t].players[p].secsTillUnpenalised
    return msg
