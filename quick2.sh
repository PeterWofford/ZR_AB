#!bin/bash
sleep 4
roslaunch astrobee spawn_ring.launch ns:=ring1 pose:="3 0.5 4.9 .7071 0 0 .7071"
roslaunch astrobee spawn_ring.launch ns:=ring2 pose:="1 -0.5 4.9 -.7071 0 0 .7071"