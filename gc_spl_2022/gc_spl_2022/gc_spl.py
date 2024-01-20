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

from rcgcd_spl_14.msg import RCGCD

from rcgcd_spl_14_conversion.conversion import rcgcd_data_to_msg
from rcgcd_spl_14_conversion.robocup_game_control_data import GAMECONTROLLER_DATA_PORT

from rcgcrd_spl_4.msg import RCGCRD

from rcgcrd_spl_4_conversion.conversion import rcgcrd_msg_to_data
from rcgcrd_spl_4_conversion.robocup_game_control_return_data import GAMECONTROLLER_RETURN_PORT


import rclpy
from rclpy.node import Node


class GCSPL(Node):
    """Node that runs on the robot to communicate with SPL GameController."""

    _loop_thread = None
    _client = None
    _publisher = None
    _host = None

    def __init__(self, node_name='gc_spl', **kwargs):
        super().__init__(node_name, **kwargs)

        # Declare parameters
        self.declare_parameter('return_port', 3939)

        # Read and log parameters
        return_port = self.get_parameter('return_port').value
        self.get_logger().debug('return_port: "%s"' % return_port)

        # Setup publisher
        self._publisher = self.create_publisher(RCGCD, 'gc/data', 10)

        # Setup subscriber
        self._subscriber = self.create_subscription(
            RCGCRD, 'gc/return_data', self._rcgcrd_callback, 10)

        # UDP Client - adapted from https://github.com/ninedraft/python-udp/blob/master/client.py
        self._client = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        # This has to be SO_REUSEADDR instead of SO_REUSEPORT to work with TCM
        self._client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
                data, (self._host, _) = self._client.recvfrom(1024)
                self.get_logger().debug('received: "%s"' % data)

                # Convert data to ROS msg
                msg = rcgcd_data_to_msg(data)

                # Publish it
                self._publisher.publish(msg)
            except TimeoutError:
                pass

    def _rcgcrd_callback(self, msg):

        if self._host is None:
            self.get_logger().debug(
                'Not returning RoboCupGameControlReturnData, as GameController'
                ' host address is not known yet.')
            return

        data = rcgcrd_msg_to_data(msg)

        # Return data directly to the GameController's address and return port
        self._client.sendto(data, (self._host, GAMECONTROLLER_RETURN_PORT))


def main(args=None):
    rclpy.init(args=args)
    gc_spl = GCSPL()
    rclpy.spin(gc_spl)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
