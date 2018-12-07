#!/bin/bash
NFPATH=~/ZR_Astrobee/game_v_0.0/src/main/java/gov/nasa/arc/astrobee/ros/java_test_square_trajectory/TestSquareTrajectoryMain.java	#path to student's java file
#begin added
DBPATH=~/ZR_AB_fakeDB/
STRNM="TEST"

cd $DBPATH
rosbag record /loc/truth/pose /loc/truth/twist /ring1/loc/truth/pose /ring2/loc/truth/pose -O $STRNM --duration=20	#records topics to this filename for given dur
rostopic echo /loc/truth/pose -b ${STRNM}.bag -p > pose.csv
rostopic echo /loc/truth/twist -b ${STRNM}.bag -p > twist.csv
rostopic echo /ring1/loc/truth/pose -b ${STRNM}.bag -p > ring1.csv
rostopic echo /ring2/loc/truth/pose -b ${STRNM}.bag -p > ring2.csv
rm ${STRNM}.bag
python createJson.py
#end edded
sleep 3
cd ~/ZR_Astrobee/game_v_0.0/
./gradlew --stop
killall roslaunch
rm $NFPATH	# gets rid of the newly made file