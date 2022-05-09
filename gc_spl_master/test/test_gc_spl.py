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

import socket
import time

from construct import Container

from gc_spl_master import robocup_game_control_data as rgcd
from gc_spl_master.gc_spl import GCSPL

from rcgcd_14.msg import RCGCD

from rcgcrd_4.msg import RCGCRD

import rclpy


class TestGCSPL:
    """Tests against GCSPL."""

    received = None

    msg = rgcd.RoboCupGameControlData.build(
        Container(
            packetNumber=1,
            playersPerTeam=5,
            competitionPhase=rgcd.COMPETITION_PHASE_PLAYOFF,
            competitionType=rgcd.COMPETITION_TYPE_7V7,
            gamePhase=rgcd.GAME_PHASE_NORMAL,
            state=rgcd.STATE_READY,
            setPlay=rgcd.SET_PLAY_CORNER_KICK,
            firstHalf=1,
            kickingTeam=1,
            secsRemaining=10,
            secondaryTime=2,
            teams=[
                Container(
                    teamNumber=1,
                    teamColour=rgcd.TEAM_BLACK,
                    score=1,
                    penaltyShot=2,
                    singleShots=3,
                    messageBudget=1234,
                    players=[
                        Container(penalty=rgcd.PENALTY_MANUAL,
                                  secsTillUnpenalised=30),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0)
                    ]),
                Container(
                    teamNumber=2,
                    teamColour=rgcd.TEAM_BLUE,
                    score=0,
                    penaltyShot=0,
                    singleShots=0,
                    messageBudget=0,
                    players=[
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0),
                        Container(penalty=rgcd.PENALTY_NONE,
                                  secsTillUnpenalised=0)
                    ])
            ]
        )
    )

    def _callback_msg(self, msg):
        self.received = msg

    def test_receiving(self):
        """
        Test GCSPL node receives from gamecontroller and publishes onto ros topic.

        This test mimics the gamecontroller sending a packet on UDP port 3838, and checks if the
        GCSPL receives it, converts it to a RCGCD message, and publishes it on the 'gc/data' topic.
        """
        # Setup nodes
        rclpy.init()
        gc_spl_node = GCSPL()  # noqa: F841
        test_node = rclpy.node.Node('test')
        subscription = test_node.create_subscription(  # noqa: F841
            RCGCD, 'gc/data', self._callback_msg, 10)

        # UDP Server - adapted from https://github.com/ninedraft/python-udp/blob/master/server.py
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            server.sendto(self.msg, ('', 3838))

        # Wait for RCGCD to receive packet over UDP, and send back a ROS msg
        time.sleep(0.1)

        # Check if message has been received
        rclpy.spin_once(test_node, timeout_sec=0)
        assert self.received is not None

        # Shutdown
        rclpy.shutdown()

    def test_sending(self):
        # Setup nodes
        rclpy.init()
        gc_spl_node = GCSPL()  # noqa: F841
        test_node = rclpy.node.Node('test')
        publisher = test_node.create_publisher(RCGCRD, 'gc/return_data', 10)

        # UDP Server - adapted from https://github.com/ninedraft/python-udp/blob/master/server.py
        server = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        server.bind((socket.gethostbyname(socket.gethostname()), 3939))
        server.settimeout(0.1)

        # Send first message so client knows GC's address.
        server.sendto(self.msg, ('', 3838))

        # Publish RCGCRD to gc_spl_node
        publisher.publish(RCGCRD())

        # Wait before spinning for the msg arrive in gc_spl_node's subscription
        time.sleep(0.01)

        # Spin gc_spl_node to process incoming message and send out UDP message
        rclpy.spin_once(gc_spl_node, timeout_sec=0)

        # Check if packet has arrived
        try:
            _ = server.recv(1024)
        except TimeoutError:
            assert False

        # Shutdown
        rclpy.shutdown()
