package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

import gov.nasa.arc.astrobee.types.Quaternion;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

public class SQuaternion {
    public double x = 0;
    public double y = 0;
    public double z = 0;
    public double w = 1;
    private double[] quat_mat = {x, y, z, w};
    private static final SVector origAstrobeeNorm = new SVector(1,0 ,0);


    private static final URI ROS_MASTER_URI = URI.create("http://localhost:11311");
    private static RingNode ringNode = null;
    private static SVector lead_vec_orig = new SVector(0, 0.6, 0);

    public static void exec(NodeMainExecutor nodeMainExecutor) {
        ringNode = new RingNode();

        // Setting configurations for ROS Node
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic("10.0.3.15");
        nodeConfiguration.setMasterUri(ROS_MASTER_URI);

        nodeMainExecutor.execute(ringNode, nodeConfiguration);
    }

    public SQuaternion() {
    }

    /**
     * @param yaw - rotation about the Y-AXIS
     * @param pitch - rotation about the Z-AXIS
     * @param roll - rotation about the X-AXIS
     * IN Y, Z, X ORDER!!!
     */
    public SQuaternion(double yaw, double pitch, double roll) {
        yaw = Math.toRadians(yaw / 2);
        pitch = Math.toRadians(pitch / 2);
        roll = Math.toRadians(roll / 2);

        x = Math.sin(yaw) * Math.sin(pitch) * Math.cos(roll) + Math.cos(yaw) * Math.cos(pitch) * Math.sin(roll);
        y = Math.sin(yaw) * Math.cos(pitch) * Math.cos(roll) + Math.cos(yaw) * Math.sin(pitch) * Math.sin(roll);
        z = Math.cos(yaw) * Math.sin(pitch) * Math.cos(roll) - Math.sin(yaw) * Math.cos(pitch) * Math.sin(roll);
        w = Math.cos(yaw) * Math.cos(pitch) * Math.cos(roll) - Math.sin(yaw) * Math.sin(pitch) * Math.sin(roll);
        quat_mat = new double[] {x, y, z, w};
    }

    public SQuaternion(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        quat_mat = new double[] {x, y, z, w};
    }

    public SQuaternion(Quaternion q) {
        this.x = q.getX();
        this.y = q.getY();
        this.z = q.getZ();
        this.w = q.getW();
    }

    public double[] getQuat() {
        return quat_mat;
    }

    /**
     * Normalizes your quaternion
     */
    public void norm() {
        double mag = Math.sqrt(Math.pow(w, 2) + Math.pow(y, 2) + Math.pow(x, 2) + Math.pow(z, 2));
        this.x /= mag;
        this.y /= mag;
        this.z /= mag;
        this.w /= mag;
        this.quat_mat = new double[] {x, y, z, w};
    }

    public String toString() {
        return "("+ this.x + ", "+ this.y + ". "+ this.z + ", "+ this.w + ")";
    }

    /**
     * Method for the quaternion it will take to convert from base ab norm vec
     * to the desired normal vector
     *
     * IF REQUESTING NORM OF -1 0 0 aka dot returns ~ -1, WILL RETURN QUAT FOR MAKING AB ROTATE ABOUT Z AXIS 180
     *
     * @param desired is the vector you want to reach
     * @return SQuaternion object that specifies correct rotation quaternion for the task
     */
    public static SQuaternion vecDiffToQuat(SVector desired) {
        return vecDiffToQuat(origAstrobeeNorm, desired);
    }

    /**
     * method used for
     * @param origNorm
     * @param desired
     * @return
     */
    public static SQuaternion vecDiffToQuat(SVector origNorm, SVector desired) {
        SQuaternion q = new SQuaternion(0, 0, 0, 1);
        desired.normalize();
        if (SVector.dot(origNorm, desired) > 0.999) {
            return new SQuaternion(0, 0, 0, 1);
        } else if (SVector.dot(origNorm, desired) < -0.999) {
            return new SQuaternion(0, 0, 1, 0);
        } else {
            SVector cross = origNorm.cross(desired);
            q.x = cross.x;
            q.y = cross.y;
            q.z = cross.z;
            q.w = Math.sqrt(Math.pow(desired.length(), 2)) + SVector.dot(origNorm, desired);
            q.norm();
        }
        return q;
    }

    public double[] quatToAxisAngles() {
        double[] axisAngles = {0, 0, 0, 0};
        if (this.w > 1) this.norm(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
        axisAngles[0] = 2 * Math.toDegrees(Math.acos(this.w));
        double s = Math.sqrt(1-this.w*this.w); // assuming quaternion normalised then w is less than 1, so term always positive.
        if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
            // if s close to zero then direction of axis not important
            axisAngles[1] = this.x; // if it is important that axis is normalised then replace with x=1; y=z=0;
            axisAngles[2] = this.y;
            axisAngles[3] = this.z;
        } else {
            axisAngles[1] = this.x / s; // normalise axis
            axisAngles[2] = this.y / s;
            axisAngles[3] = this.z / s;
        }
        return axisAngles;
    }
    
    public SVector quatToEuler() {
        SVector v = new SVector();
        double sqw = this.w*this.w;
        double sqx = this.x*this.x;
        double sqy = this.y*this.y;
        double sqz = this.z*this.z;
        double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
        double test = this.x*this.y + this.z*this.w;
        if (test > 0.499*unit) { // singularity at north pole
            v.x = 2 * Math.atan2(this.x,this.w);
            v.y = Math.PI/2;
            v.z = 0;
        } else if (test < -0.499*unit) { // singularity at south pole
            v.x = -2 * Math.atan2(this.x,this.w);
            v.y = -Math.PI/2;
            v.z = 0;
        } else {
            v.x = Math.atan2(2 * this.y * this.w - 2 * this.x * this.z, sqx - sqy - sqz + sqw);
            v.y = Math.asin(2 * test / unit);
            v.z = Math.atan2(2 * this.x * this.w - 2 * this.y * this.z, -sqx + sqy - sqz + sqw);
        }
        v.x = Math.toDegrees(v.x);
        v.y = Math.toDegrees(v.y);
        v.z = Math.toDegrees(v.z);
        // yaw, pitch, roll; or heading, attitude, bank; or Y, Z, X rotations, in that order
        return v;
    }

    public static SQuaternion getRing1Quat() {
        return ringNode.ring1_orient;
    }

    public static SQuaternion getRing2Quat() {
        return ringNode.ring2_orient;
    }

    public static SVector rotateVecByQuat(SVector in, SQuaternion rotation) {
        // Normalize first
        rotation.norm();
        // Extract vector part of quat
        SVector quat_vec = new SVector(rotation.x, rotation.y, rotation.z);

        // Extract scalar part of quat
        double scalar = rotation.w;

        SVector prod1 = quat_vec.scalarMultiply(2 * quat_vec.dot(in) ); //2*dot(in,q) * q
        SVector prod2 = in.scalarMultiply(Math.pow(scalar, 2) - quat_vec.dot(quat_vec));
        SVector prod3 = quat_vec.cross(in).scalarMultiply(2 * scalar); //2*s*cross(in, q)

        SVector sum = prod1.add(prod2).add(prod3);
        return sum;
    }

    public SVector leadVec(SQuaternion rotation) {
        return SQuaternion.rotateVecByQuat(lead_vec_orig, rotation);
    }

    public static void main(String... args) throws InterruptedException {
/*
        SQuaternion quat = new SQuaternion( 0, 0, 90);
        SQuaternion other = new SQuaternion(0,0,0,1);
        SVector desired_norm = new SVector(1, 0, -1);
        SQuaternion q = other.vecDiffToQuat(new SVector(1,0,0), desired_norm);
        double[] val = q.getQuat();
        for (double x: val) {
            System.out.println(x);
        }


        exec(DefaultNodeMainExecutor.newDefault());
        Thread.sleep(3000);
        for (int i = 0; i < 10; i++) {
            SVector v = SQuaternion.getRing1Quat().quatToEuler();
            System.out.println("Euler ring is ::" + v);
            SVector v3 = SQuaternion.getRing2Quat().quatToEuler();
            System.out.println("Euler ring2 is ::" + v3);
            //System.out.println(ringNode.getRing2Quat());
            Thread.sleep(670);
        }

        /*
        SQuaternion q2 = new SQuaternion(.7071, 0, 0, .7071);
        SVector v2 = q2.quatToEuler();
        System.out.println(v2);
        System.out.println(SQuaternion.vecDiffToQuat(new SVector(0, -1, 0)));

        SVector original = new SVector(0, 0.6, 0);
        SQuaternion rotation = new SQuaternion(-0.7071, 0, 0, 0.7071);
        SVector result = SQuaternion.rotateVecByQuat(original, rotation);
        System.out.println(result);
        */
    }
}
