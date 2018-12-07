#!/bin/bash
SPEED=1.0
echo "LAUNCHING ASTROBEE SIMULATION AT ${SPEED} TIMES SPEED"
source ~/freeflyer_build/native/devel/setup.bash
~/ZR_Astrobee/ringlaunch.sh &
roslaunch astrobee sim.launch sviz:=true speed:=${SPEED}
