# lgsvl_simulator

[LG SVL Simulator](https://github.com/lgsvl/simulator) package for [AutowareArchitectureProposal](https://github.com/tier4/AutowareArchitectureProposal.proj)

# Requirements

* [AutowareArchitectureProposal](https://github.com/tier4/AutowareArchitectureProposal.proj/tree/ros1) with branch ros1

* [LG SVL Simulator](https://github.com/lgsvl/simulator/releases/tag/2021.3) with version 2021.3

* [autoware_msgs](https://github.com/autocore-ai/autoware_msgs) in Autoware.AI

# How to use

1. After clone and build [AutowareArchitectureProposal](https://github.com/tier4/AutowareArchitectureProposal.proj/tree/ros1) with branch ros1 successfully, clone [this package](https://github.com/autocore-ai/lgsvl_simulator) and [autoware_msgs](https://github.com/autocore-ai/autoware_msgs) in Autoware.AI.
    ```bash
    AutowareArchitectureProposal.proj$ git clone git@github.com:autocore-ai/lgsvl_simulator.git src/simulator/lgsvl_simulator

    AutowareArchitectureProposal.proj$ git clone git@github.com:autocore-ai/autoware_msgs.git src/simulator/autoware_msgs
    ```
1. Build this package
    ```bash
    AutowareArchitectureProposal.proj$ colcon build -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --packages-select autoware_msgs lgsvl_simulator
    ```
1. Launch LG SVL Simulator and config case with:
    * Map: AutonomouStuff
    * Vehicle: Lexus2016RXHybird
    * Sensor configuration: Autoware AI
    * Bridge: ROS
    * Autopilot: Other ROS Autopilot
    * Connection: 127.0.0.1:9090
1. Edit firewall rule add port `9090` to whitelist
1. Launch LG SVL Simulator and start play, then drive car on road by hand.
1. Launch AutowareArchitectureProposal with this package launch and drag goal pose on Rviz.
    ```bash
    AutowareArchitectureProposal.proj$ source install/setup.bash
    AutowareArchitectureProposal.proj$ roslaunch lgsvl_simulator autoware.launch
    ```
