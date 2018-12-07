#!/bin/bash
sleep 3
# GO_ON="false"
source ~/freeflyer_build/native/devel/setup.bash
roslaunch astrobee spawn_ring.launch ns:=ring1 pose:="3.0 0.5 4.9 0.7071 0.0 0.0 0.7071"
roslaunch astrobee spawn_ring.launch ns:=ring2 pose:="1.0 -0.5 4.9 -0.7071 0.0 0.0 0.7071"
# here to add running shuttles, 1 and 2
bash ~/ZR_Astrobee/shuttler.sh

sleep 3
echo "------going on after shuttler"

if test -z "$GO_ON"
then	# var is empty
	echo "-------CONTINUING TO SIMULATE"
	cd ~/ZR_Astrobee/game_v_0.0
	./gradlew build
	echo "----- RUNNING THE GAME"
	~/ZR_Astrobee/launchend.sh &
	./gradlew run
	
else	# var not empty
	echo "---------WE STOPPING HERE CHIEF"
	killall roslaunch
fi
