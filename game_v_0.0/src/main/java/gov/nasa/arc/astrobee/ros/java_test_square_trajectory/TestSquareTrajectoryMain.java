package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Result;

import static java.lang.Thread.*;


public class TestSquareTrajectoryMain {

    private static Thread t;

    private static ApiCommandImplementation astrobee = ApiCommandImplementation.getInstance();
    private static ApiCommandImplementation.ZR_API api= null; // astrobee.new zrAPI();
    private static ApiCommandImplementation.Game_API game = null; // astrobee.new Game_API();
    private static PlayerTemplate PlayerCode;

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
    private static Point[] arrayPoint = { POINT_3, POINT_4, POINT_4, POINT_4, POINT_3, POINT_4, POINT_4, POINT_4, POINT_4};
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

        // Get a unique instance of the ab_info in order to access info about the robot
        ABInfo abInfo = ABInfo.getABInfoInstance();

        // Get unique instances of zr and game api's for allowing Players to command bot
        api  = ApiCommandImplementation.get_zr_api();
        game = ApiCommandImplementation.get_game_api();

        // Give Player Thread access to api's
        PlayerCode = new PlayerTemplate(api, game);

//        Starting threads
        Point destination = new Point(0,0.6,5.1);
        Quaternion quat = new Quaternion(0.707f,0f,0f,0.707f);
//        astrobee.moveToValid(destination,quat);
        astrobee.executionThread();
        System.out.println("Started Astrobee Thread!");
        RunPlayerThread();
        System.out.println("Started Player Thread!");


        // Loop the points and orientation previously defined.
        /*
        for(int i = 0; i < 100; i++) {
            System.out.println("TIME");
            System.out.println(.getCurrentTime());
            Thread.sleep(1000);
        }

        // The below is fallback example
        for (int i = 0; i < arrayPoint.length; i++) {
            System.out.println("attempting to move to:: " + SPoint.toSPoint(arrayPoint[i]) + " with quat:: " + arrayOrient[i]);
            System.out.println("another loop");
            System.out.println(astrobee.moveToValid(arrayPoint[i], arrayOrient[i]));    //make sure movetovalid public
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
    }
    static int i = 0;
    public static void RunPlayerThread() {

        if (t == null) {
            t = new Thread() {
                public void run() {
                    while(true) {
                        if (i == 0) {        // testing movement
                            WayPoint test1 = new WayPoint(3, 0, 4.9, 0.707, 0, 0, 0.707);
                            i++;
                            System.out.println(api);
                            api.addWayPoint(test1);
//                        System.out.println("Added WayPoint");
                        }
                        try {
                            System.out.println("starting player code");
                            PlayerCode.loop();              // calls the body of their loop
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
