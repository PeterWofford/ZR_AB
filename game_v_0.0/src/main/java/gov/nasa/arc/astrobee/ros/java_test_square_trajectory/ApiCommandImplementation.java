
/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

import gov.nasa.arc.astrobee.*;
import gov.nasa.arc.astrobee.ros.DefaultRobotFactory;
import gov.nasa.arc.astrobee.ros.RobotConfiguration;
import gov.nasa.arc.astrobee.types.FlashlightLocation;
import gov.nasa.arc.astrobee.types.PlannerType;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import org.apache.commons.lang.StringUtils;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.node.DefaultNodeMainExecutor;

import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Collections;
//import java.util.concurrent.TimeUnit;
//import java.util.concurrent.TimeoutException;
//import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.*;

/**
 * A simple API implementation for sending commands to the Astrobee for an interactive game.
 */

public class ApiCommandImplementation {
    private static final Log logger = LogFactory.getLog(ApiCommandImplementation.class);

    // The IP for the ROS Master
    private static final URI ROS_MASTER_URI = URI.create("http://10.42.0.1:11311");
    // IP for the local computer executing this code
    private static final String JAVA_ROS_HOSTNAME = "10.42.0.1";
    // Name of the node
    private static final String NODE_NAME = "astrobee_java_app";

    private static final int SUCCESS = -1;
    private static final int VALIDATE_ERROR = 0;
    private static final int MOVE_TO_ERROR = 1;


    // VARS FOR RING SCORING RETURNS
    private static final int SCORE_RING_ONE = 1;
    private static final int SCORE_RING_TWO = 2;
    private static final int MISS_ERROR = 0;
    private static final int NOT_IN_RING_ERROR = -1;
    private static final int FLASHLIGHT_ERROR = -2;
    private Thread t = null;

    // The instance to access this class
    private static ApiCommandImplementation instance = null;

    // Configuration that will keep data to connect with ROS master
    private RobotConfiguration robotConfiguration = new RobotConfiguration();

    // Instance that will create a robot with the given configuration
    private RobotFactory factory;

    // The robot itself
    private Robot robot;

    private PlannerType plannerType = null;
    private FlashlightLocation flashlight;

    private GameManager game = new GameManager();


    //The Game Variables
    /* Astrobee should start at 2, 0, 4.9 */
    // AB info object only hold info about pollen collection and general game logic stuffs
    private static ABInfo myAstrobeeInfo = ABInfo.getABInfoInstance();

    private static Timer time = new Timer();
    private static double start_time = 0.0;

    private static SQuaternion orient_manager = new SQuaternion(0, 0, 0, 1);

    /* Keep Out Zones */
    private final KeepOutZoneRingWPlants test_ring_1 = new KeepOutZoneRingWPlants(new SPoint(3,0.5,4.9),
            0.6, 0.2, new SVector(0,-1,0), Math.PI / 4);
    /*  ^^ Quaternion of [0.7071, 0, 0, 0.7071] ^^      */
    private final KeepOutZoneRingWPlants test_ring_2 = new KeepOutZoneRingWPlants(new SPoint(1,-0.5,4.9),
            0.6, 0.2,  new SVector(0,1,0), Math.PI / 4);
    /*  ^^ Quaternion of [-0.7071, 0, 0, 0.7071] ^^    */
    private final KeepOutZoneRingWPlants[] ringsWPlants = { test_ring_1, test_ring_2 };

    /* Game score */
    private int score = 0;

    /*WayPoint Queue*/
    private List<WayPoint> WaypointQueue;

    public int getScore() {
        return ABInfo.getScore();
    }

    /**
     * Private constructor that prevents other objects from creating instances of this class.
     * Instances of this class must be provided by a static function (Singleton)
     */
    private ApiCommandImplementation() {

        /* Alternative custom configuration
         *
         * configureRobot();
         * factory = new DefaultRobotFactory(robotConfiguration);
         *
         */

        factory = new DefaultRobotFactory();
        WaypointQueue = Collections.synchronizedList(new ArrayList<WayPoint>());

        try {
            // Get the robot
            robot = factory.getRobot();

            Kinematics k = getTrustedRobotKinematics();

            logger.info("Position: " + k.getPosition());

            // Set default planner
            setPlanner(PlannerType.TRAPEZOIDAL);

        } catch (AstrobeeException e) {
            logger.info("Error with Astrobee");
        } catch (InterruptedException e) {
            logger.info("Connection Interrupted");
        }

        time.exec(DefaultNodeMainExecutor.newDefault());
        orient_manager.exec(DefaultNodeMainExecutor.newDefault());
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        ScoreManager.initial();
    }

    /**
     * Static method that provides a unique instance of this class
     *
     * @return A unique instance of this class ready to use
     */
    public static ApiCommandImplementation getInstance() {
        if (instance == null) {
            instance = new ApiCommandImplementation();
        }
        return instance;
    }

    /**
     * This method sets a default configuration for the robot
     */
    private void configureRobot() {
        // Populating robot configuration
        robotConfiguration.setMasterUri(ROS_MASTER_URI);
        robotConfiguration.setHostname(JAVA_ROS_HOSTNAME);
        robotConfiguration.setNodeName(NODE_NAME);
    }

    /**
     * This method shutdown the robot factory in order to allow java to close correctly.
     */
    private void shutdownFactory() {
        factory.shutdown();
    }

    private Result getCommandResult(PendingResult pending, boolean printRobotPosition) {

        Result result = null;

        try {
            Kinematics k;
            //SPoint rollpitchyaw = new SPoint(0, 0, 0);
            //SPoint plant = new SPoint(1, 0, 4.8);
            //Plants xyz = new Plants(1, 1.5, plant);


            // Waiting until command is done.
            while (!pending.isFinished()) {
                if (printRobotPosition) {
                    // Meanwhile, let's get the positioning along the trajectory

                    //k = robot.getCurrentKinematics();
                    /*
                    //rollpitchyaw = rollpitchyaw.quat_rpy(k.getOrientation());

                    System.out.println(k.getPosition().toString());
                    //System.out.println(xyz.rpy_cone(rollpitchyaw).toString());

                    //SPoint plantvec = xyz.plant_vec(plant.toSPoint(k.getPosition()), plant);
                    //System.out.println(plantvec.toString());

                    //System.out.print(xyz.score(plantvec, xyz.rpy_cone(rollpitchyaw)));
                    System.out.println("-----");
                    */


                    //logger.info("Current Position: " + k.getPosition().toString());
                    //logger.info("Current Orientation" + k.getOrientation().toString());
                }

                // Wait a little bit before retry
                pending.getResult(1000, TimeUnit.MILLISECONDS);
            }

            // Getting final result
            result = pending.getResult();

            // Print result in the log.
            printLogCommandResult(result);

        } catch (AstrobeeException e) {
            logger.info("Error with Astrobee");
        } catch (InterruptedException e) {
            logger.info("Connection Interrupted");
        } catch (TimeoutException e) {
            logger.info("Timeout connection");
        } finally {
            // Return command execution result.
            return result;
        }
    }

    /**
     * Get trusted data related to the motion, positioning and orientation for Astrobee
     *
     * @return
     */
    private Kinematics getTrustedRobotKinematics() {
        logger.info("Waiting for robot to acquire position");

        // Variable that will keep all data related to positioning and movement.
        Kinematics k;

        // Waiting until we get a trusted kinematics
        while (true) {
            // Get kinematics
            k = robot.getCurrentKinematics();

            // Is it good?
            if (k.getConfidence() == Kinematics.Confidence.GOOD)
                // Don't wait anymore, move on.
                break;

            // It's not good, wait a little bit and try again
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                logger.info("It was not possible to get a trusted kinematics. Sorry");
                return null;
            }
        }

        return k;
    }

    /**
     * It moves Astrobee to the given point and rotate it to the given orientation.
     *
     * @param goalPoint   Absolute cardinal point (xyz)
     * @param orientation An instance of the Quaternion class.
     *                    You may want to use INITIAL_POSITION as an example.
     * @return A Result instance carrying data related to the execution.
     * Returns null if the command was NOT execute as a result of an error
     */
    public Result moveTo(Point goalPoint, Quaternion orientation) {

        // First, stop all motion
        Result result = stopAllMotion();
        if (result.hasSucceeded()) {
            // We stopped, do your stuff now

            // Setting a simple movement command using the end point and the end orientation.
            PendingResult pending = robot.simpleMove6DOF(goalPoint, orientation);

            // Get the command execution result and send it back to the requester.
            result = getCommandResult(pending, true);
        }
        return result;
    }


    public class ZR_API {
        public double[] getOrientation() {
            SQuaternion q =  new SQuaternion(getTrustedRobotKinematics().getOrientation());
            return q.getQuat();
        }

        public double[] getPosition() {
            SPoint p = SPoint.toSPoint(getTrustedRobotKinematics().getPosition());
            return p.getMy_coords();
        }
        public void add(WayPoint wp){
            ApiCommandImplementation.this.WaypointQueue.add(wp);
        }
    }
    public class Game_API {
        public void setAttitudeTarget(double x, double y, double z) {
            SVector target = new SVector(x, y, z);
            setAttitudeTarget(target);
        }

        public void setAttitudeTarget(SVector target) {
            ApiCommandImplementation.this.setAttitudeTarget(target);
        }
        //TODO: Implement Ring info methods

    }

    public SQuaternion getOrientation() {
        return new SQuaternion(getTrustedRobotKinematics().getOrientation());
    }

    public SPoint getPosition() {
        return SPoint.toSPoint(getTrustedRobotKinematics().getPosition());
    }

    /**
     * Takes in a direction vector and sets it as target
     * @param x vec compoment in world x
     * @param y vec component in world y
     * @param z vec component in world z
     */
    private void setAttitudeTarget(double x, double y, double z) {
        SVector target = new SVector(x, y, z);
        setAttitudeTarget(target);
    }

    private void setAttitudeTarget(SVector target) {
        target.normalize();
        SQuaternion orient = SQuaternion.vecDiffToQuat(target);
        Quaternion qOrient = new Quaternion((float) orient.x, (float) orient.y, (float) orient.z, (float) orient.w);
        moveToValid(getTrustedRobotKinematics().getPosition(), qOrient);
    }

    /**
     * Checks if the Astrobee destination will work given the KOZs
     * @param goalPoint (xyz) point you want to test if valid
     *
     * @return A bool, false if AB will collide, true if not
     */
    public boolean validWaypoint(Point goalPoint){
        SPoint cur_pos = SPoint.toSPoint(getTrustedRobotKinematics().getPosition());
        SPoint goal = new SPoint(goalPoint.getX(), goalPoint.getY(), goalPoint.getZ());
        ArrayList<Map<Integer, List<Object>>> results = new ArrayList<Map<Integer, List<Object>>>();
        ArrayList<Integer> projections = new ArrayList<Integer>();
        int koz_count = ringsWPlants.length;
        for (int i = 0; i < koz_count; i++) {
            // iterates through the keepOutZones to check any collisions
            results.add(ringsWPlants[i].aB_path_projection(cur_pos, goal));
        }
        for (Map<Integer, List<Object>> result : results) {
            for (Map.Entry<Integer, List<Object>> entry : result.entrySet()) {
                projections.add((Integer) entry.getValue().get(0));
            }
        }
        if (projections.contains(0)) {
            /*
            System.out.println("returns were" + projections);
            System.out.println("movement failed");
            */
            return false;
        } else {
            /*
            System.out.println("returns were" + projections);
            */
            return true;
        }
    }

    /**
     * It moves Astrobee to the given point and rotate it to the given orientation.
     * IFF valid wayPoint returns true
     *
     * @param goalPoint   Absolute cardinal point (xyz)
     * @param orientation An instance of the Quaternion class.
     *                    You may want to use INITIAL_POSITION as an example.
     * @return An int corresponding to the result of the action.
     */
    public int moveToValid(Point goalPoint, Quaternion orientation) {
        if (!validWaypoint(goalPoint)) {
            return VALIDATE_ERROR;
        } else {
            Result movement_result = moveTo(goalPoint, orientation);
            if (!movement_result.hasSucceeded()) {
                return MOVE_TO_ERROR;
            } else {
                return SUCCESS;
            }
        }
    }

    private double getCurrentTime(){
        System.out.println(start_time);
        double currT  = time.getTime();
        System.out.println(currT);
        double currT_adjusted = currT - start_time;
        return currT_adjusted;
    }

    public Result stopAllMotion() {
        PendingResult pendingResult = robot.stopAllMotion();
        return getCommandResult(pendingResult, false);
    }

    /**
     * TODO :: Make this return an array, first value being status (score, outside ring, flashlight error, etc), second being
     * TODO :: ring, third being plant
     *
     * @return just an int for now
     * @throws InterruptedException (in case flashlight fails or gets interrupted)
     */
    // COMMENTED OUT BECAUSE WE SWITCHED TO CONNECTING TO RING NODES THEMSELVES
/*
    public int pollinate() {

        Kinematics k;
        k = getTrustedRobotKinematics();

        int result;

        SPoint pos = SPoint.toSPoint(k.getPosition());
        SPoint[] ring_centers = new SPoint[keepOutZones.length];
        for (int i = 0; i < keepOutZones.length; i++)
            ring_centers[i] = keepOutZones[i].get_center();
        double[] distsSquared = pos.distSquared(ring_centers);
        SPoint rpy = SPoint.quat_rpy(k.getOrientation());

        if (distsSquared[0] <= Math.pow(KeepOutZone.getAB_collider_radius(), 2)) {
            SPoint lead = plants1.plant_vec(initial_lead_plant_pos_1, SPoint.toSPoint(k.getPosition()));
            SPoint[] spawned = plants1.spawn_plants(lead, getCurrentTime());


            for(int i = 0; i < plants1.getPlant_number(); i++) {
                boolean score = plants1.score(spawned[i], plants1.rpy_cone(rpy));
                if(score){
                    this.score = Plants.decide_score(i, this.score);
                }
            }
            System.out.print("CURRENT SCORE: ");
            System.out.println(this.score);
            result = SCORE_RING_ONE;

        } else if (distsSquared[1] <= Math.pow(KeepOutZone.getAB_collider_radius(), 2)) {
            SPoint lead = plants2.plant_vec(initial_lead_plant_pos_2, SPoint.toSPoint(k.getPosition()));
            SPoint[] spawned = plants2.spawn_plants(lead, getCurrentTime());

            for(int i = 0; i < plants2.getPlant_number(); i++) {
                boolean score = plants2.score(spawned[i], plants2.rpy_cone(rpy));
                if(score){
                    this.score = Plants.decide_score(i, this.score);
                }
            }
            System.out.print("CURRENT SCORE: ");
            System.out.println(this.score);
            result = SCORE_RING_TWO;
        } else {
            System.out.println("NOT IN RING");
            game.score -= 50;
            result = NOT_IN_RING_ERROR;
        }
        try {
            Result flashlight = flashlight_shine();
        } catch (InterruptedException e) {
            System.out.println("Interrupted Exception!");
            result = FLASHLIGHT_ERROR;
        }
        return result;
    }
*/

    /*
    *   returns an int code of result for each ring
    *   -1 - no score
    *   1 - score
    *   also modifies the game score accordingly and keeps track of pollination attempts
     */

    /**
     * Executes the pollination command for the Astrobee, shines the flashlight and returns an
     * integer of the result, codes being 0 for misfire, -1 for not in ring, -2 for flashlight error
     * if ring integer, tells the ring number scored in
     * @return the integer code result
     */
    private int pollinate() {

        /*  Kinematic object for accessing astrobee pose */
        Kinematics k;
        k = getTrustedRobotKinematics();

        SPoint abPos = SPoint.toSPoint(k.getPosition());
        SQuaternion abOrient = new SQuaternion(k.getOrientation());

        int kozLength = ringsWPlants.length;
        /*  array for holding results for each ring
        *   with 0 being out of ring, 1 being successful,
        *   -1 being a miss */
        int[] results = new int[kozLength];

        /*  counter for if in ring */
        int outOfRingCount = 0;

        /*  increments the pollination attempts */
        ABInfo.incrementAttempts();

        for (int i = 0; i < kozLength; i++) {
            double distSquared = abPos.distSquared(ringsWPlants[i].get_center());
            if (!(distSquared <= Math.pow(ABInfo.collider_radius, 2))) {
                // too far away from ring center
                results[i] = 0;
                outOfRingCount++;
            } else {
                SQuaternion ringOrient;
                if (i == 0) {
                    ringOrient = SQuaternion.getRing1Quat();
//                    System.out.println(ringOrient);
                } else if (i == 1) {
                    ringOrient = SQuaternion.getRing2Quat();
                } else break; // if greater than numRings break!!

                // newPollen being collected / given to, prevPollen is donor
                String newPollen = ringsWPlants[i].scoreOnRing(abOrient, ringOrient);
                String prevPollen = ABInfo.getPollenType();

                // handles if successful hit
                if (StringUtils.isEmpty(newPollen)){
                    results[i] = -1;
                    ABInfo.changeScore(Pollen.getMissPenalty());
                    ScoreManager.updatem(Pollen.getMissPenalty(), 0);
                }else{
                    results[i] = 1;
                    ABInfo.incrementSuccess();
                    ScoreManager.updateh(false, newPollen);
                }

                // handles scoring logic in terms of pollen types
                if (StringUtils.isNotEmpty(prevPollen) && StringUtils.isNotEmpty(newPollen)) {
                    if (Pollen.prevCanGiveTo(prevPollen, newPollen)) {
                        ABInfo.changeScore(Pollen.getPollinateScore(newPollen));
                        ScoreManager.updateh(true, newPollen);
                    } else {
                        ABInfo.changeScore(Pollen.getMispollinatePenalty());
                        ScoreManager.updatem(0, Pollen.getMispollinatePenalty());
                    }
                }
                ABInfo.setPollenType(newPollen);
                ABInfo.changeScore(Pollen.getCollectionScore(newPollen));
            }
        }


        ScoreManager.updateratio(ABInfo.getScore(), ABInfo.getPollinateSuccesses(), ABInfo.getPollinateAttempts());
        ScoreManager.updategui();


        /*  handles the shining of the flashlight  */
        try {
            flashlightShine();
        } catch (InterruptedException e) {
            e.printStackTrace();
            /* if the flashlight did not shine, want to return this */
            return FLASHLIGHT_ERROR;
        }

        /*  returns the appropriate scoring if flashlight shined correctly  */
        for (int r = 0; r < results.length; r++) {
            if (results[r] == 1) {
                if (r == 0){
                    return SCORE_RING_ONE;
                } else
                    return SCORE_RING_TWO;
            }
        }

//        if (outOfRingCount == kozLength) {
//            System.out.println("NOT IN ANY RINGS");
//            return NOT_IN_RING_ERROR;
//        }
        return MISS_ERROR;
    }

    /**
     * Method for shining the flashlight
     * @return Whether or not the flashlight succesfully turned on
     * @throws InterruptedException
     */
    private Result flashlightShine() throws InterruptedException {

        robot.setFlashlightBrightness(FlashlightLocation.FRONT, 1);

        Thread.sleep(100);

        PendingResult pending = robot.setFlashlightBrightness(FlashlightLocation.FRONT, 0);

        Result result = getCommandResult(pending, false);

        return result;
    }

    /**
     * An optional method used to print command execution results on the Android log
     * @param result
     */
    private void printLogCommandResult(Result result) {
        logger.info("Command status: " + result.getStatus().toString());

        // In case command fails
        if (!result.hasSucceeded()) {
            logger.info("Command message: " + result.getMessage());
        }

        logger.info("Done");
    }

    /**
     * Method to get the robot from this API Implementation.
     * @return
     */
    public Robot getRobot() {
        return robot;
    }

    public boolean setPlanner(PlannerType plannerType) {
        PendingResult pendingPlanner = robot.setPlanner(plannerType);
        Result result = getCommandResult(pendingPlanner, false);
        if (result.hasSucceeded()) {
            this.plannerType = plannerType;
            logger.info("Planner set to " + plannerType);
        }

        return result.hasSucceeded();
    }

    private void execute(){
        if (WaypointQueue.size() > 0){
            WayPoint executing = WaypointQueue.remove(0);
            double[] coords = executing.get_waypoint_point();
            Point destination = new Point(coords[0],coords[1],coords[2]);
            double[] angles = executing.get_waypoint_quat();
            Quaternion quat = new Quaternion((float)(angles[0]),(float)(angles[1]),(float)(angles[2]),(float)(angles[3]));
            moveTo(destination, quat);
        }else{
            System.out.println("Queue is Empty!");
        }


    }

    public void runThread(){
        if (t == null) {
            t = new Thread() {
                public void run() {
                    while(true) {
                        try {
                            System.out.println("Loop");
                            //                        this.setAttitudeTarget(iHat);
                            //                        this.setAttitudeTarget(n_kHat);
                            //                        this.setAttitudeTarget(jHat);
                            //                        this.setAttitudeTarget(kHat);
                            //                        this.setAttitudeTarget(n_jHat);
                            execute();
                            Thread.sleep(1000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
            };
        }
        t.start();
    }
}
