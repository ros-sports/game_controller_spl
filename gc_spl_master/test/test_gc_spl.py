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

from gc_spl_master.gc_spl import GCSPL

from rcgcd_14.msg import RCGCD

import rclpy


class TestGCSPL:
    """Tests against GCSPL."""

    received = None

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
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            server.sendto(b'Hello world!', ('', 3838))

        # Wait before spinning for the ROS msg to be sent from GCSPL to arrive in the subscription
        time.sleep(0.01)

        # Check if message has been received
        rclpy.spin_once(test_node, timeout_sec=0)
        assert self.received is not None

        # Shutdown
        rclpy.shutdown()
