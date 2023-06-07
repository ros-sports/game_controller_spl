# GC SPL

ROS2 Package to interface RoboCup Standard Platform League's Game Controller.

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=humble)](../../actions/workflows/build_and_test_humble.yaml?query=branch:humble)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_iron.yaml?query=branch:rolling)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)
[![Build and Test (dev)](../../actions/workflows/build_and_test_dev.yaml/badge.svg?branch=dev)](../../actions/workflows/build_and_test_dev.yaml?query=branch:dev)

For more information, see our [Documentation](https://gamecontroller-spl.readthedocs.io/)

# Branches

Note that this repository has an **unusual branching strategy**.
The default branch is the **dev** branch.

The **dev** branch is unreleased, and should work with SPL GameController's master branch.
When a new game controller release is made (eg. 2023, GermanOpen2023), the package should be backported to active distros (eg. rolling, humble, etc.) and released.

That way, teams don't have to be on the most recent distro to use a package against a recent GameController!
