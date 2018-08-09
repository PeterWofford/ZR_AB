package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

import java.lang.reflect.Array;
import java.util.ArrayList;

// A simple vector class, modified from MIT CSAIL
class SVector {
    public double x, y, z;

    // constructors
    public SVector() {
    }

    public SVector(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public SVector(SVector v) {
        x = v.x;
        y = v.y;
        z = v.z;
    }

    public SVector(SPoint p) {
        x = p.get_x();
        y = p.get_y();
        z = p.get_z();
    }

    // methods

    // Vector from this to B
    public final SVector to(SVector B) {
        return (new SVector(B.x - x, B.y - y, B.z - z));
    }

    public final double dot(SVector B) {
        return (x * B.x + y * B.y + z * B.z);
    }

    public final double dot(double Bx, double By, double Bz) {
        return (x * Bx + y * By + z * Bz);
    }

    public static final double dot(SVector A, SVector B) {
        return (A.x * B.x + A.y * B.y + A.z * B.z);
    }

    public final SVector cross(SVector B) {
        return new SVector(y * B.z - z * B.y, z * B.x - x * B.z, x * B.y - y * B.x);
    }

    public final SVector cross(double Bx, double By, double Bz) {
        return new SVector(y * Bz - z * By, z * Bx - x * Bz, x * By - y * Bx);
    }

    public final static SVector cross(SVector A, SVector B) {
        return new SVector(A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x);
    }

    public final double length() {
        return (double) Math.sqrt(x * x + y * y + z * z);
    }

    public final static double length(SVector A) {
        return (double) Math.sqrt(A.x * A.x + A.y * A.y + A.z * A.z);
    }

    public static ArrayList<Double> lengths(ArrayList<SVector> vecs){
        ArrayList<Double> mags = new ArrayList<>();
        for (SVector vec : vecs){
            mags.add(vec.length());
        }
        return mags;
    }

    public final void normalize() {
        double t = x * x + y * y + z * z;
        if (t != 0 && t != 1) t = (double) (1 / Math.sqrt(t));
        x *= t;
        y *= t;
        z *= t;
    }

    public final static SVector normalize(SVector A) {
        double t = A.x * A.x + A.y * A.y + A.z * A.z;
        if (t != 0 && t != 1) t = (double) (1 / Math.sqrt(t));
        return new SVector(A.x * t, A.y * t, A.z * t);
    }

    public static void setMagnitude(SVector a, double magnitude) {
        a.normalize();
        a.x *= magnitude;
        a.y *= magnitude;
        a.z *= magnitude;
    }

    public SVector setMagnitude(double magnitude) {
        SVector change = new SVector(this);
        change.normalize();
        change.x *= magnitude;
        change.y *= magnitude;
        change.z *= magnitude;
        return change;
    }


    public SVector scalarMultiply(double scalar){
        SVector multiplied = new SVector(this);
        multiplied.x *= scalar;
        multiplied.y *= scalar;
        multiplied.z *= scalar;
        return multiplied;
    }

    public SVector negate(){
        double neg_x = -1 * this.x;
        double neg_y = -1 * this.y;
        double neg_z = -1 * this.z;
        return new SVector(neg_x, neg_y, neg_z);
    }

    public SVector add(SVector other) {
        double new_x = this.x + other.x;
        double new_y = this.y + other.y;
        double new_z = this.z + other.z;
        return new SVector(new_x, new_y, new_z);
    }

    public String toString() {
        return new String("[" + SPoint.round(x, 3) + ", " + SPoint.round(y, 3) + ", " + SPoint.round(z, 3) + "]");
    }

    public boolean equals(SVector other) {
        if(x == other.x && y == other.y && z == other.z)
            return true;
        return false;
    }

    public static double scalar_projection(SVector vec1, SVector vec2){

        //PROJECTION OF VEC2 ONTO VEC1
        if(vec1.length() != 0.0) {
            double scalar_p = vec1.dot(vec2) / vec1.length();
            return scalar_p;
        }else{
            System.out.println("IMPROPER VECTOR PROJECTION: DIVISION BY ZERO");
            return 0.0;
        }
    }

    /**
     * Assumes input vector is not zero
     * gives a unit vector normal to your current vector
     * @param in SVector you want a normal vector to
     * @return unit SVector normal to input vector
     */
    public SVector getVecNormalTo(SVector in) {
        SVector inNorm = normalize(in);
        SVector iHat = new SVector(1,0,0);
        if (!(inNorm.equals(iHat) || inNorm.equals(iHat.negate()))) {
            return normalize(inNorm.cross(iHat));
        }
        SVector jHat = new SVector(0, 1, 0);
        return normalize(inNorm.cross(jHat));
    }


    /**
     * @param norm - normal vector to be rotated about
     * @param vec - vector to be rotated
     * @param theta - angle amount to rotate by (in radians)
     * @return SVector of newly rotated vector
     */
    public static SVector rodriguezRotation(SVector norm, SVector vec, double theta) {
        SVector e1 = vec.scalarMultiply(Math.cos(theta));
        SVector e2 = SVector.cross(norm, vec).scalarMultiply(Math.sin(theta));
        SVector e3 = norm.scalarMultiply(SVector.dot(norm, vec)*(1-Math.cos(theta)));

        SVector vrot = e1.add(e2).add(e3);
        return vrot;
    }

    public static SVector genVec(SPoint a, SPoint b) {
        // returns vector from b to a
        double x = a.get_x() - b.get_x();
        double y = a.get_y() - b.get_y();
        double z = a.get_z() - b.get_z();
        return new SVector(x, y, z);
    }

    public static ArrayList<SVector> genVecs (ArrayList<SPoint> points, SPoint target) {
        // returns vectors from target to points
        ArrayList<SVector> vecs  = new ArrayList<SVector>();
        for (SPoint point : points){
            vecs.add(genVec(point, target));
        }
        return vecs;
    }

    public double getAngleDeg(SVector compare){
        double dot = dot(this, compare);
        double mag = this.length() * compare.length();
        if (mag != 0){
            double acos = Math.toDegrees(Math.acos(dot / mag));
            return acos;
        }
        return -1.0;
    }

    public static ArrayList<Double> getAnglesDeg(ArrayList<SVector> vecs, SVector compare){
        ArrayList<Double> degs = new ArrayList<Double>();
        for (SVector vec : vecs){
            degs.add(vec.getAngleDeg(compare));
        }
        return degs;
    }
    /*
    public static void main(String[] args) {
        SPoint pretend_AB_pos = new SPoint(2.4, 0, 0);
        SPoint AB_goal = new SPoint(-2, 0.5, -1);
        SPoint koz_center = new SPoint(0, 0, 0);
        SVector norm = new SVector(1, 0, 0);
        ArrayList<SPoint> points = pretend_AB_pos.splitPath(AB_goal);
        ArrayList<SVector> vecs = SVector.genVecs(points, koz_center);
        ArrayList<Double> angles = SVector.getAnglesDeg(vecs, norm);
        int i = vecs.size();
        for (int x = 0; x < i; x++){
            System.out.println(vecs.get(x) +" "+ angles.get(x));
        }
    }
    */
}
