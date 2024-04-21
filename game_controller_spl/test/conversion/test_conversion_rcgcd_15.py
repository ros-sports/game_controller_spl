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

from game_controller_spl.rcgcd_15.conversion import rcgcd_data_to_msg

from game_controller_spl.rcgcd_15.robocup_game_control_data import RoboCupGameControlData
from game_controller_spl_interfaces.msg import RCGCD15 as RCGCD
from game_controller_spl_interfaces.msg import RobotInfo15 as RobotInfo
from game_controller_spl_interfaces.msg import TeamInfo15 as TeamInfo


def test_rcgcd_data_to_msg():
    """Test conversion of binary data to RCGCD msg."""
    data = RoboCupGameControlData.build(
        Container(
            packetNumber=1,
            playersPerTeam=5,
            competitionPhase=RCGCD.COMPETITION_PHASE_PLAYOFF,
            competitionType=RCGCD.COMPETITION_TYPE_DYNAMIC_BALL_HANDLING,
            gamePhase=RCGCD.GAME_PHASE_PENALTYSHOOT,
            state=RCGCD.STATE_READY,
            setPlay=RCGCD.SET_PLAY_GOAL_KICK,
            firstHalf=1,
            kickingTeam=1,
            secsRemaining=10,
            secondaryTime=2,
            teams=[
                Container(
                    teamNumber=1,
                    fieldPlayerColour=TeamInfo.TEAM_RED,
                    goalkeeperColour=TeamInfo.TEAM_RED,
                    goalkeeper=1,
                    score=1,
                    penaltyShot=2,
                    singleShots=3,
                    messageBudget=1000,
                    players=[
                        Container(penalty=RobotInfo.PENALTY_MANUAL,
                                  secsTillUnpenalised=1),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_SPL_PLAYER_PUSHING,
                                  secsTillUnpenalised=2),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                    ]),
                Container(
                    teamNumber=2,
                    fieldPlayerColour=TeamInfo.TEAM_YELLOW,
                    goalkeeperColour=TeamInfo.TEAM_YELLOW,
                    goalkeeper=1,
                    score=4,
                    penaltyShot=5,
                    singleShots=6,
                    messageBudget=2000,
                    players=[
                        Container(penalty=RobotInfo.PENALTY_MANUAL,
                                  secsTillUnpenalised=11),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_SPL_PLAYER_PUSHING,
                                  secsTillUnpenalised=12),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=RobotInfo.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                    ])
            ]
        )
    )

    msg = rcgcd_data_to_msg(data)
    assert msg.packet_number == 1
    assert msg.players_per_team == 5
    assert msg.competition_phase == RCGCD.COMPETITION_PHASE_PLAYOFF
    assert msg.competition_type == RCGCD.COMPETITION_TYPE_DYNAMIC_BALL_HANDLING
    assert msg.game_phase == RCGCD.GAME_PHASE_PENALTYSHOOT
    assert msg.state == RCGCD.STATE_READY
    assert msg.set_play == RCGCD.SET_PLAY_GOAL_KICK
    assert msg.first_half == 1
    assert msg.kicking_team == 1
    assert msg.secs_remaining == 10
    assert msg.secondary_time == 2
    assert msg.teams[0].team_number == 1
    assert msg.teams[0].field_player_colour == TeamInfo.TEAM_RED
    assert msg.teams[0].goalkeeper_colour == TeamInfo.TEAM_RED
    assert msg.teams[0].goalkeeper == 1
    assert msg.teams[0].score == 1
    assert msg.teams[0].penalty_shot == 2
    assert msg.teams[0].single_shots == 3
    assert msg.teams[0].message_budget == 1000
    assert msg.teams[0].players[0].penalty == RobotInfo.PENALTY_MANUAL
    assert msg.teams[0].players[0].secs_till_unpenalised == 1
    assert msg.teams[0].players[6].penalty == RobotInfo.PENALTY_SPL_PLAYER_PUSHING
    assert msg.teams[0].players[6].secs_till_unpenalised == 2
    assert msg.teams[1].team_number == 2
    assert msg.teams[1].field_player_colour == TeamInfo.TEAM_YELLOW
    assert msg.teams[1].goalkeeper_colour == TeamInfo.TEAM_YELLOW
    assert msg.teams[1].goalkeeper == 1
    assert msg.teams[1].score == 4
    assert msg.teams[1].penalty_shot == 5
    assert msg.teams[1].single_shots == 6
    assert msg.teams[1].message_budget == 2000
    assert msg.teams[1].players[0].penalty == RobotInfo.PENALTY_MANUAL
    assert msg.teams[1].players[0].secs_till_unpenalised == 11
    assert msg.teams[1].players[6].penalty == RobotInfo.PENALTY_SPL_PLAYER_PUSHING
    assert msg.teams[1].players[6].secs_till_unpenalised == 12
