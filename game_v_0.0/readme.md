# ZR trajectory project

This project is based off an example meant to be a quick introduction 
to the Astrobee API Development. This project has been wrote to be 
independent of the API source. In order to do that we use jar libraries 
to import the code for the Astrobee API and Freeflyer ROS messages.

*Note the APIs may change in time. Ensure to have the ultimate version of the
jar libraries.*

## Running the example

### Building the code

First, we build the example. Assuming that this repository is checked out
under `ZR_AB` :

    you@machine:~ $ cd $ZR_AB/game_v_0.0
    you@machine:game_v_0.0 $ ./gradlew build

### Setup environmental variables

Assuming you have a proper build of the Astrobee flight software located in
`$BUILD_PATH`, the following will setup your environment to run the simulator
locally:

    you@machine:~ $ . $BUILD_PATH/devel/setup.bash
    you@machine:~ $ export ROS_IP=127.0.0.1
    you@machine:~ $ export ROS_MASTER_URI=http://${ROS_IP}:11311/

### Start the Astrobee simulator

In one terminal, with the environment variables setup:

    you@machine:~ $ roslaunch astrobee sim.launch sviz:=true
    ...

### (Option 1) Run the example using Gradle

In another terminal, with the environment setup properly:

    export ROS_MASTER_URI=http://${ROS_IP}:11311/
    you@machine:~ $ cd $ZR_AB/game_v_0.0
    you@machine:game_v_0.0 $ ./gradlew run
    ...

### (Option 2) Run the example using Java VM

#### Generating jar file

In another terminal, with the environment setup properly:

    export ROS_MASTER_URI=http://${ROS_IP}:11311/
    you@machine:~ $ cd $ZR_AB/game_v_0.0
    you@machine:game_v_0.0 $ ./gradlew jar
    ...

#### Run file

In the same terminal and assuming you have Java installed. Execute:

    java -jar build/libs/game_v_0.0-1.0-SNAPSHOT.jar

## Importing into IntelliJ IDEA

 * Open IntelliJ and click `Import Project`.
 * Navigate to the root of the `ZR_AB` directory (this directory)
 * Select `Import project from external model`
 * Select `Gradle`
 * Click `Next`
 * Uncheck `Use auto-import`
 * Uncheck `Create directories...`
 * Check `Create separate module per source set`
 * Uncheck `Store generated project files externally`
 * Select `Use default gradle wrapper`
 * Select `.idea (Directory Based)` Project format
 * Click `Finish`

