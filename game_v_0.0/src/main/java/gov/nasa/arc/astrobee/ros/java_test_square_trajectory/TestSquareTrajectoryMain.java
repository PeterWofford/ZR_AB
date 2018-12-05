package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Result;

import static java.lang.Thread.*;


public class TestSquareTrajectoryMain {

    private static Thread t;

    private static ApiCommandImplementation astrobee = ApiCommandImplementation.getInstance();
    private static ApiCommandImplementation.ZR_API api= astrobee.new ZR_API();
    private static ApiCommandImplementation.Game_API game = astrobee.new Game_API();
    private static PlayerTemplate PlayerCode = new PlayerTemplate();

    // Fixed trajectory points
    private static final Point HOME_POSITION = new Point(2, 0, 4.9);
    private static final Point POINT_1 = new Point(1, 0, 4.9);
    private static final Point POINT_2 = new Point(1, -0.5, 4.9);
    private static final Point POINT_3 = new Point(3, 0, 4.9);
    private static final Point POINT_4 = new Point(3, 0.5, 4.9);
    private static final Point POINT_5 = new Point(0, 0.6, 5.1);

    // Fixed trajectory orientations (POINT_1 and 2 use default orientation)
    private static final Quaternion DEFAULT_ORIENT = new Quaternion();
    private static final Quaternion X_POS_ROLL = new Quaternion(0.707f, 0, 0, 0.707f);
    private static final Quaternion X_NEG_ROLL = new Quaternion(-0.707f, 0, 0, 0.707f);
    private static final Quaternion UP_FACING = new Quaternion(0, -0.707f, 0, 0.707f);
    private static final Quaternion DOWN_FACING = new Quaternion(0, 0.707f, 0, 0.707f);
    private static final Quaternion LEFT_FACING = new Quaternion(0, 0, 0.707f, 0.707f);
    private static final Quaternion RIGHT_FACING = new Quaternion(0, 0, -0.707f, 0.707f);
    private static final Quaternion BACK_FACING = new Quaternion(0, 0, 1, 0);
    private static final Quaternion ORIENT_1 = new Quaternion(0, -0.3827f, 0, 0.9239f);
    private static final Quaternion ORIENT_2 = new Quaternion(0, -0.9239f, 0, 0.3827f);
    private static final Quaternion ORIENT_3 = new Quaternion(0, 0.9239f, 0, 0.3827f);
    private static final Quaternion ORIENT_4 = new Quaternion(0, 0.3827f, 0, 0.9239f);

    // Defining trajectory. Fixed positions and orientations. An orientation for each position.
    private static Point[] arrayPoint = { POINT_3, POINT_4, POINT_4, POINT_4, POINT_4, POINT_4, POINT_4, POINT_4, POINT_4};
    private static Quaternion[] arrayOrient = {LEFT_FACING, DEFAULT_ORIENT, ORIENT_1, DEFAULT_ORIENT, ORIENT_2, DEFAULT_ORIENT, ORIENT_3, DEFAULT_ORIENT, ORIENT_4};

    private static final SVector iHat = new SVector(1, 0, 0);
    private static final SVector jHat = new SVector(0, 1, 0);
    private static final SVector kHat = new SVector(0, 0, 1);
    private static final SVector n_iHat = new SVector(-1, 0, 0);
    private static final SVector n_jHat = new SVector(0, -1, 0);
    private static final SVector n_kHat = new SVector(0, 0, -1);


    public static void main(String[] args) throws InterruptedException {
        // Because log4j doesn't do the needful
        setDefaultUncaughtExceptionHandler(new UnhandledExceptionHandler());

        // Get a unique instance of the Astrobee API in order to command the robot.
        ABInfo abInfo = ABInfo.getABInfoInstance();

        Result result;
        //Starting threads
        Point destination = new Point(0,0.6,5.1);
        Quaternion quat = new Quaternion(0.707f,0f,0f,0.707f);
        astrobee.moveTo(destination,quat);
//        astrobee.runThread();
//        System.out.println("Started Astrobee Thread!");
//        RunPlayerThread();
//        System.out.println("Started Player Thread!");

//        astrobee.setAttitudeTarget(iHat);
//        astrobee.setAttitudeTarget(n_kHat);
//        astrobee.setAttitudeTarget(jHat);
//        astrobee.setAttitudeTarget(kHat);
//        astrobee.setAttitudeTarget(n_jHat);
//        astrobee.setAttitudeTarget(n_iHat);


        // Loop the points and orientation previously defined.
        /*
        for(int i = 0; i < 100; i++) {
            System.out.println("TIME");
            System.out.println(api.getCurrentTime());
            Thread.sleep(1000);
        }


        for (int i = 0; i < arrayPoint.length; i++) {
            System.out.println("attempting to move to:: " + SPoint.toSPoint(arrayPoint[i]) + " with quat:: " + arrayOrient[i]);
            System.out.println("another loop");
            System.out.println(api.moveToValid(arrayPoint[i], arrayOrient[i]));
            int counter = 3;
            for (int c = 0; c < counter; c++) {
                if ( c % 2 == 1) {
                    System.out.println(api.pollinate());
                    System.out.println(api.getScore());
                }
                //System.out.println(api.getCurrentTime());
                sleep(1000);
            }
        }
        */


        /* Will print the elapsed time it took for the calls to execute above */
       // System.out.println("This is the amount of time it took::" + startUpTest.timeElapsed(System.currentTimeMillis()));

        // Stop the API
//        astrobee.shutdownFactory();
    }
    static int i = 0;
    public static void RunPlayerThread() {

        if (t == null) {
            t = new Thread() {
                public void run() {
                    if (i == 0){
                        WayPoint test1 = new WayPoint(0,0.6,5.1,0.707,0,0,0.707);
//                        WayPoint base = new WayPoint();
                        i++;
                        api.add(test1);
//                        api.add(base);
                        System.out.println("Added WayPoint");
                    }
                    try {
                        PlayerCode.loop();
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            };
        }
        t.start();
    }
}
