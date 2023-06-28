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

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class GCSPL(Node):
    """Node that runs on the robot to communicate with SPL GameController."""

    _loop_thread = None
    _client = None
    _publisher = None
    _host = None
    _RCGCD = None

    def __init__(self, node_name='gc_spl', **kwargs):
        super().__init__(node_name, **kwargs)

        # Declare parameters
        self.declare_parameter('return_port', 3939)
        self.declare_parameter('rcgcd_version', Parameter.Type.INTEGER)
        self.declare_parameter('rcgcrd_version', Parameter.Type.INTEGER)

        # Read and log parameters
        return_port = self.get_parameter('return_port').value
        self.get_logger().debug('return_port: %s' % return_port)

        # Import libraries dependending on rcgcd and rcgcrd version
        rcgcd_version = self.get_parameter('rcgcd_version').value
        if rcgcd_version is None:
            self.get_logger().error('rcgcd_version not provided as a parameter. ' +
                                    'Unable to start node.')
            return
        self.get_logger().debug('rcgcd_version: %s' % rcgcd_version)

        rcgcrd_version = self.get_parameter('rcgcrd_version').value
        if rcgcrd_version is None:
            self.get_logger().error('rcgcrd_version not provided as a parameter. ' +
                                    'Unable to start node.')
            return
        self.get_logger().debug('rcgcrd_version: %s' % rcgcrd_version)

        self._setup_methods(rcgcd_version, rcgcrd_version)

        # Setup publisher
        self._publisher = self.create_publisher(self.RCGCD, 'gc/data', 10)

        # Setup subscriber
        self._subscriber = self.create_subscription(
            self.RCGCRD, 'gc/return_data', self._rcgcrd_callback, 10)

        # UDP Client - adapted from https://github.com/ninedraft/python-udp/blob/master/client.py
        self._client = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        # This has to be SO_REUSEADDR instead of SO_REUSEPORT to work with TCM
        self._client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._client.bind(('', self.GAMECONTROLLER_DATA_PORT))
        # Set timeout so _loop can constantly check for rclpy.ok()
        self._client.settimeout(0.1)

        # Start thread to continuously poll
        self._loop_thread = Thread(target=self._loop)
        self._loop_thread.start()

    def _setup_methods(self, rcgcd_version, rcgcrd_version):
        # RCGCD
        if rcgcd_version == 14:
            from gc_spl_interfaces.msg import RCGCD14 as RCGCD
            from gc_spl.rcgcd_14.robocup_game_control_data import \
                GAMECONTROLLER_DATA_PORT
            from gc_spl.rcgcd_14.conversion import rcgcd_data_to_msg
        elif rcgcd_version == 15:
            from gc_spl_interfaces.msg import RCGCD15 as RCGCD
            from gc_spl.rcgcd_15.robocup_game_control_data import \
                GAMECONTROLLER_DATA_PORT
            from gc_spl.rcgcd_15.conversion import rcgcd_data_to_msg
        else:
            self.get_logger().error('rcgcd_version ' + rcgcd_version + ' is not supported.')
        self.RCGCD = RCGCD
        self.GAMECONTROLLER_DATA_PORT = GAMECONTROLLER_DATA_PORT
        self.rcgcd_data_to_msg = rcgcd_data_to_msg

        # RCGCRD
        if rcgcrd_version == 4:
            from gc_spl_interfaces.msg import RCGCRD4 as RCGCRD
            from gc_spl.rcgcrd_4.conversion import rcgcrd_msg_to_data
            from gc_spl.rcgcrd_4.robocup_game_control_return_data import \
                GAMECONTROLLER_RETURN_PORT
        else:
            self.get_logger().error('rcgcrd_version ' + rcgcrd_version + ' is not supported.')
        self.RCGCRD = RCGCRD
        self.rcgcrd_msg_to_data = rcgcrd_msg_to_data
        self.GAMECONTROLLER_RETURN_PORT = GAMECONTROLLER_RETURN_PORT

    def _loop(self):
        while rclpy.ok():
            try:
                data, (self._host, _) = self._client.recvfrom(1024)
                self.get_logger().debug('received: "%s"' % data)

                # Convert data to ROS msg
                msg = self.rcgcd_data_to_msg(data)

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

        data = self.rcgcrd_msg_to_data(msg)

        # Return data directly to the GameController's address and return port
        self._client.sendto(data, (self._host, self.GAMECONTROLLER_RETURN_PORT))


def main(args=None):
    rclpy.init(args=args)
    gc_spl = GCSPL()
    rclpy.spin(gc_spl)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
