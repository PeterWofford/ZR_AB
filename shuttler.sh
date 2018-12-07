#!bin/bash
javac shuttle_one.java
java shuttle_one		# puts the student code into the dir for running
javac shuttle_two.java
java shuttle_two
javac shuttle_three.java
java shuttle_three
echo "---- file PlayerTemplate.java should be inside game now! ---"
rm ~/ZR_AB_fakeDB/PlayerTemplate2.java
rm ~/ZR_AB_fakeDB/PlayerTemplate3.java