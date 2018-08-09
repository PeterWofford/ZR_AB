package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

import gov.nasa.arc.astrobee.*;
import gov.nasa.arc.astrobee.ros.DefaultRobotFactory;
import gov.nasa.arc.astrobee.ros.RobotConfiguration;
import gov.nasa.arc.astrobee.types.PlannerType;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class ZR_API {

    private static ZR_API instance = null;
    private static final Log logger;

    // Configuration that will keep data to connect with ROS master
    private RobotConfiguration robotConfiguration = new RobotConfiguration();

    // Instance that will create a robot with the given configuration
    private RobotFactory factory;

    // The robot itself
    private Robot robot;

    private PlannerType plannerType = null;

    static {
        logger = LogFactory.getLog(ApiCommandImplementation.class);
    }

    /**
     * Static method that provides a unique instance of this class
     *
     * @return A unique instance of this class ready to use
     */
    public static ZR_API getInstance() {
        if (instance == null) {
            instance = new ZR_API();
        }
        return instance;
    }

    /**
     * Private constructor that prevents other objects from creating instances of this class.
     * Instances of this class must be provided by a static function (Singleton)
     */
    private ZR_API() {

        /* Alternative custom configuration
         *
         * configureRobot();
         * factory = new DefaultRobotFactory(robotConfiguration);
         *
         */

        factory = new DefaultRobotFactory();

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

        //game = new GameManager(1, 1.5, new SPoint(1, 0, 4.5));
    }

    /**
     * Get trusted data related to the motion, positioning and orientation for Astrobee
     *
     * @return
     */
    public Kinematics getTrustedRobotKinematics() {
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

    public boolean setPlanner(PlannerType plannerType) {
        PendingResult pendingPlanner = robot.setPlanner(plannerType);
        Result result = getCommandResult(pendingPlanner, false);
        if (result.hasSucceeded()) {
            this.plannerType = plannerType;
            logger.info("Planner set to " + plannerType);
        }

        return result.hasSucceeded();
    }

    public Result getCommandResult(PendingResult pending, boolean printRobotPosition) {

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

    ApiCommandImplementation api = ApiCommandImplementation.getInstance();


}
