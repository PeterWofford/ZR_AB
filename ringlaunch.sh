#!/bin/bash
sleep 3
source ~/freeflyer_build/native/devel/setup.bash
roslaunch astrobee spawn_ring.launch ns:=ring1 pose:="3.0 0.5 4.9 0.7071 0.0 0.0 0.7071"
roslaunch astrobee spawn_ring.launch ns:=ring2 pose:="1.0 -0.5 4.9 -0.7071 0.0 0.0 0.7071"
# here to add running shuttle
javac shuttle.java
java shuttle		# puts the student code into the dir for running
cd ~/ZR_Astrobee/game_v_0.0
./gradlew build
~/ZR_Astrobee/launchend.sh &
./gradlew run