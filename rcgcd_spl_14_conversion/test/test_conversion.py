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

from rcgcd_spl_14_conversion.conversion import rcgcd_data_to_msg

from rcgcd_spl_14_conversion.robocup_game_control_data import (
    COMPETITION_PHASE_PLAYOFF, COMPETITION_TYPE_CHALLENGE_SHIELD,
    GAME_PHASE_PENALTYSHOOT, PENALTY_MANUAL, PENALTY_NONE, PENALTY_SPL_PLAYER_PUSHING,
    RoboCupGameControlData, SET_PLAY_GOAL_KICK, STATE_READY, TEAM_RED, TEAM_YELLOW)


def test_rcgcd_data_to_msg():
    """Test conversion of binary data to RCGCD msg."""
    data = RoboCupGameControlData.build(
        Container(
            packetNumber=1,
            playersPerTeam=5,
            competitionPhase=COMPETITION_PHASE_PLAYOFF,
            competitionType=COMPETITION_TYPE_CHALLENGE_SHIELD,
            gamePhase=GAME_PHASE_PENALTYSHOOT,
            state=STATE_READY,
            setPlay=SET_PLAY_GOAL_KICK,
            firstHalf=1,
            kickingTeam=1,
            secsRemaining=10,
            secondaryTime=2,
            teams=[
                Container(
                    teamNumber=1,
                    teamColour=TEAM_RED,
                    score=1,
                    penaltyShot=2,
                    singleShots=3,
                    messageBudget=1000,
                    players=[
                        Container(penalty=PENALTY_MANUAL,
                                  secsTillUnpenalised=1),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_SPL_PLAYER_PUSHING,
                                  secsTillUnpenalised=2)
                    ]),
                Container(
                    teamNumber=2,
                    teamColour=TEAM_YELLOW,
                    score=4,
                    penaltyShot=5,
                    singleShots=6,
                    messageBudget=2000,
                    players=[
                        Container(penalty=PENALTY_MANUAL,
                                  secsTillUnpenalised=11),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=PENALTY_SPL_PLAYER_PUSHING,
                                  secsTillUnpenalised=12)
                    ])
            ]
        )
    )

    msg = rcgcd_data_to_msg(data)
    assert msg.packet_number == 1
    assert msg.players_per_team == 5
    assert msg.competition_phase == COMPETITION_PHASE_PLAYOFF
    assert msg.competition_type == COMPETITION_TYPE_CHALLENGE_SHIELD
    assert msg.game_phase == GAME_PHASE_PENALTYSHOOT
    assert msg.state == STATE_READY
    assert msg.set_play == SET_PLAY_GOAL_KICK
    assert msg.first_half == 1
    assert msg.kicking_team == 1
    assert msg.secs_remaining == 10
    assert msg.secondary_time == 2
    assert msg.teams[0].team_number == 1
    assert msg.teams[0].team_colour == TEAM_RED
    assert msg.teams[0].score == 1
    assert msg.teams[0].penalty_shot == 2
    assert msg.teams[0].single_shots == 3
    assert msg.teams[0].message_budget == 1000
    assert msg.teams[0].players[0].penalty == PENALTY_MANUAL
    assert msg.teams[0].players[0].secs_till_unpenalised == 1
    assert msg.teams[0].players[6].penalty == PENALTY_SPL_PLAYER_PUSHING
    assert msg.teams[0].players[6].secs_till_unpenalised == 2
    assert msg.teams[1].team_number == 2
    assert msg.teams[1].team_colour == TEAM_YELLOW
    assert msg.teams[1].score == 4
    assert msg.teams[1].penalty_shot == 5
    assert msg.teams[1].single_shots == 6
    assert msg.teams[1].message_budget == 2000
    assert msg.teams[1].players[0].penalty == PENALTY_MANUAL
    assert msg.teams[1].players[0].secs_till_unpenalised == 11
    assert msg.teams[1].players[6].penalty == PENALTY_SPL_PLAYER_PUSHING
    assert msg.teams[1].players[6].secs_till_unpenalised == 12
