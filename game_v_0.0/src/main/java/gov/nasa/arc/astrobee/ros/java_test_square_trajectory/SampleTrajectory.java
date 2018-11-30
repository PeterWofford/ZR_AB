package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class SampleTrajectory {
    private static final Point HOME_POSITION = new Point(2, 0, 4.8);
    private static final Point UP = new Point(2, 0, 5.1);
    private static final Point POINT_1 = new Point(1, 0.6, 4.8);
    private static final Point POINT_2 = new Point(5, 0.6, 5.1);
    private static final Point POINT_3 = new Point(5, -0.3, 4.8);
    private static final Point POINT_4 = new Point(1, -0.3, 4.8);
    private static final Point POINT_5 = new Point(1, 0.6, 5.1);

    // Fixed trajectory orientations (POINT_1 and 2 use default orientation)
    private static final Quaternion DEFAULT_ORIENT = new Quaternion();
    private static final Quaternion ORIENT_3 = new Quaternion(0, 0, -0.707f, 0.707f);
    private static final Quaternion ORIENT_4 = new Quaternion(0, 0, 1, 0);

    // Defining trajectory. Fixed positions and orientations. An orientation for each position.
    private static Point[] arrayPoint = {POINT_1, POINT_2, POINT_3, POINT_4, POINT_1, POINT_5};
    private static Quaternion[] arrayOrient = {DEFAULT_ORIENT, ORIENT_3, ORIENT_4, DEFAULT_ORIENT, DEFAULT_ORIENT};

    public static void main(String[] args) {
        // Because log4j doesn't do the needful
        Thread.setDefaultUncaughtExceptionHandler(new UnhandledExceptionHandler());

        // Get a unique instance of the Astrobee API in order to command the robot.
        ApiCommandImplementation api = ApiCommandImplementation.getInstance();

        // At the end of execution, this variable will contain the last command result. It may be useful
        Result result = null;


        //api.pollinate(FlashlightLocation.BACK, 0, 0);
        //api.pollinate(FlashlightLocation.FRONT, 0, 0);

        // Loop the points and orientation previously defined.
        /*
        for (int i = 0; i < arrayOrient.length; i++) {
            result = api.moveTo(HOME_POSITION, arrayOrient[i]);
            if (!result.hasSucceeded()) {
                // If any movement fails we cancel all execution.
                break;
            }else {
                try {
                    api.pollinate();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        //api.moveTo(POINT_2, DEFAULT_ORIENT);
        */

        // Stop the API
        // api.shutdownFactory();
    }
}
