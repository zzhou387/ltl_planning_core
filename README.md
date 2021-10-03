# ltl_planning_core

## Overview
This repo contains a multi-robot task planner based on LTL (Linear Temporal Logic). An interface to BT (Behavior Tree)
is also provided. The whole codebase is developed upon a ROS package from [ltl_automaton_core](https://github.com/KTH-SML/ltl_automaton_core).

## Installation

### Dependencies
- [Robot Operating System (ROS)](http://wiki.ros.org)(duh), package tested on Kinetic, Melodic and Morenia distributions

- [LTL2BA](https://github.com/KTH-DHSG/ros_ltl2ba). ROS package wrapping for the LTL2BA software by Dennis Oddoux and Paul Gastin.
    - Clone the repository from Github in your catkin workspace:
    ```
    cd catkin_ws/src
    git clone https://github.com/KTH-DHSG/ros_ltl2ba
    ```
    - Build your workspace with your prefered tool:
    `cd ..`
    `catkin_make` or `catkin build`

- [PLY (Python Lex-Yacc)](http://www.dabeaz.com/ply/)
	- For Python2 (ROS Kinetic & Melodic):
	`pip install ply`
	- For Python3 (ROS Morenia):
	`pip3 install ply`

- [NetworkX](https://networkx.org/). Software for complex networks analysis in Python.
	- For Python2 (ROS Kinetic & Melodic):
	`pip install networkx`
	- For Python3 (ROS Morenia):
	`pip3 install networkx`

- [PyYAML](https://pyyaml.org/). Should be integrated with ROS but it's better to check if version is up-to-date.
	- For Python2 (ROS Kinetic & Melodic):
	`pip install pyyaml`
	- For Python3 (ROS Morenia):
	`pip3 install pyyaml`


[comment]: <> (### Building)

[comment]: <> (To build the package, clone the current repository in your catkin workspace and build it.)

[comment]: <> (```)

[comment]: <> (cd catkin_ws/src)

[comment]: <> (git clone https://github.com/KTH-SML/ltl_automaton_core.git)

[comment]: <> (```)

[comment]: <> (Build your workspace with either *catkin_make* or *catkin build*)

[comment]: <> (```)

[comment]: <> (cd ...)

[comment]: <> (catkin_make)

[comment]: <> (```)

## Usage


## Packages
This metapackage is composed of the following packages.

- **[ltl_automaton_planner](/ltl_automaton_planner)**: Provides the LTL planner node. The node uses a transition system and a LTL formula to generate a plan and action sequence, and update them according to agent state.

- **[ltl_automaton_msgs](/ltl_automaton_msgs)**: A message definition packages for the LTL automaton packages.

- **[ltl_automaton_std_transition_systems](/ltl_automaton_std_transition_systems)**: A set of state monitors for standard transition systems (2D regions, ...).
