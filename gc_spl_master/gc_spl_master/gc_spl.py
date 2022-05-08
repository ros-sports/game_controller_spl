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
from threading import Thread

from robocup_game_control_data import GAMECONTROLLER_DATA_PORT, RoboCupGameControlData

from rcgcd_14.msg import RCGCD

import rclpy
from rclpy.node import Node


class GCSPL(Node):
    """Node that runs on the robot to communicate with SPL GameController."""

    _loop_thread = None
    _client = None
    _publisher = None

    def __init__(self, node_name='gc_spl', **kwargs):
        super().__init__(node_name, **kwargs)

        # Read and log parameters
        return_port = self.get_parameter_or('return_port', 3939)
        self.get_logger().debug('return_port: "%s"' % return_port)

        # Setup publisher
        self._publisher = self.create_publisher(RCGCD, 'gc/data', 10)

        # UDP Client - adapted from https://github.com/ninedraft/python-udp/blob/master/client.py
        self._client = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        self._client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self._client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._client.bind(('', GAMECONTROLLER_DATA_PORT))
        # Set timeout so _loop can constantly check for rclpy.ok()
        self._client.settimeout(0.1)

        # Start thread to continuously poll
        self._loop_thread = Thread(target=self._loop)
        self._loop_thread.start()

    def _loop(self):
        while rclpy.ok():
            try:
                data, _ = self._client.recvfrom(1024)
                self.get_logger().debug('received: "%s"' % data)

                # Convert data to RoboCupGameControlData struct
                parsed_data = RoboCupGameControlData.parse(data)

                # Convert parsed_data to ROS msg
                rcgcd = RCGCD()

                # Publish it
                self._publisher.publish(rcgcd)
            except TimeoutError:
                pass


def main(args=None):
    rclpy.init(args=args)
    gc_spl = GCSPL()
    rclpy.spin(gc_spl)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
