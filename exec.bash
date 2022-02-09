#!/bin/bash
DELAY_CMD='read -p \"Press enter AFTER SITL starts, or mavros wil crash immediately.\"'
ROS_CMD="schroot -c bionic -- bash -c \"source ./devel/setup.bash && source gazeboenv.bash && ${DELAY_CMD} && roslaunch sitl_driver png_mission.launch\""
SITL_CMD='cd ~/ardupilot && ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 -I0'

tmux \
	new-session -s pursuit "$SITL_CMD" \; \
	split-window -h "$ROS_CMD" \; \