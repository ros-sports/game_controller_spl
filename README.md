# GC SPL

ROS2 Package to interface RoboCup Standard Platform League's Game Controller.

[![Build and Test (foxy)](https://github.com/ijnek/gc_spl/actions/workflows/build_and_test_foxy.yaml/badge.svg?branch=galactic)](https://github.com/ijnek/gc_spl/actions/workflows/build_and_test_foxy.yaml?query=branch:galactic)
[![Build and Test (galactic)](https://github.com/ijnek/gc_spl/actions/workflows/build_and_test_galactic.yaml/badge.svg?branch=galactic)](https://github.com/ijnek/gc_spl/actions/workflows/build_and_test_galactic.yaml?query=branch:galactic)
[![Build and Test (humble)](https://github.com/ijnek/gc_spl/actions/workflows/build_and_test_humble.yaml/badge.svg?branch=rolling)](https://github.com/ijnek/gc_spl/actions/workflows/build_and_test_humble.yaml?query=branch:rolling)
[![Build and Test (rolling)](https://github.com/ijnek/gc_spl/actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](https://github.com/ijnek/gc_spl/actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)


## Examples

    # Send data to the GameController
    ros2 topic pub --once /gc/return_data rcgcrd_4/msg/RCGCRD "{player_num: 1, team_num: 2}"

    # See data sent by GameController
    ros2 topic echo /gc/data