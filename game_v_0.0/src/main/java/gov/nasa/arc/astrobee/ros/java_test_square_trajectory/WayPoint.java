package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

/**
 * The data structure which will hold the WayPoint targets for the Astrobee pose, including xyz and quaternion
 * The point and quat objects are immutable to keep waypoint objects safe
 */
public class WayPoint {
    private final SPoint my_point;
    private final SQuaternion my_quat;
//    private final SVector attitude_vec;

    /**
     * DEFAULT CONSTRUCTOR
     * sets pos to initial astrobee spawn point
     */
    public WayPoint() {
        this.my_point = new SPoint(2, 0, 4.9);                              // init pose
//        this.attitude_vec = new SVector(1, 0, 0);
        this.my_quat = SQuaternion.vecDiffToQuat(new SVector(1,0 ,0));     // init quat
    }

    public WayPoint(double x, double y, double z, double q_x, double q_y, double q_z, double q_w) {
        this.my_point = new SPoint(x, y, z);
        this.my_quat = new SQuaternion(q_x, q_y, q_z, q_w);
    }

    public WayPoint(double x, double y, double z, double a_x, double a_y, double a_z) {
        this.my_point = new SPoint(x, y, z);
//        this.attitude_vec = new SVector(a_x, a_y, a_z);
        this.my_quat = SQuaternion.vecDiffToQuat(new SVector(a_x, a_y, a_z));
    }

    public double[] get_waypoint_point() {
        return my_point.getMy_coords();
    }

    public double[] get_waypoint_quat() {
        return my_quat.getQuat();
    }
}
