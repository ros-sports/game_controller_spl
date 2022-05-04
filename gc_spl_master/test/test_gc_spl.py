import rclpy
from gc_spl_master.gc_spl import GCSPL


def test_node():
    rclpy.init()
    node = GCSPL()
    rclpy.shutdown()
