#!/bin/bash
NFPATH=~/ZR_Astrobee/game_v_0.0/src/main/java/gov/nasa/arc/astrobee/ros/java_test_square_trajectory/PlayerTemplate.java	#path to student's java file
#begin added
LENRECORD=40
DBPATH=~/ZR_AB_fakeDB/
STRNM="TEST"

cd $DBPATH
echo "--------RECORDING TO ${STRNM}.BAG"
rosbag record /loc/truth/pose /loc/truth/twist /ring1/loc/truth/pose /ring2/loc/truth/pose -O $STRNM --duration=${LENRECORD}	#records topics to this filename for given dur
echo "-------WRITING BAG TO CSV OBJECTS"
rostopic echo /loc/truth/pose -b ${STRNM}.bag -p > pose.csv
rostopic echo /loc/truth/twist -b ${STRNM}.bag -p > twist.csv
rostopic echo /ring1/loc/truth/pose -b ${STRNM}.bag -p > ring1.csv
rostopic echo /ring2/loc/truth/pose -b ${STRNM}.bag -p > ring2.csv
rm ${STRNM}.bag
echo "------REMOVED ${STRNM}.BAG"
python createJson.py
echo "------WROTE CSVS TO JSON OBJECTS"
#end edded
sleep 3
echo "------KILLING GAME AND SIMULATION"
cd ~/ZR_Astrobee/game_v_0.0/
./gradlew --stop
killall roslaunch
rm $NFPATH	# gets rid of the newly made file
echo "------REMOVED PLAYER TEMPLATE FROM GAME JAVA PROJECT"