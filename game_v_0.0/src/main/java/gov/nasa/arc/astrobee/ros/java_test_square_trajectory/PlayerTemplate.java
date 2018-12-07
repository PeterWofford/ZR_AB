package gov.nasa.arc.astrobee.ros.java_test_square_trajectory;

public class PlayerTemplate{
    //Put fields here
    ApiCommandImplementation.ZR_API api;
    ApiCommandImplementation.Game_API game;
    int counter = 0;
    WayPoint des_wp = null;
    WayPoint cur_wp = null;

    //Setup
    public PlayerTemplate(ApiCommandImplementation.ZR_API api, ApiCommandImplementation.Game_API game) {
        this.api = api;
        this.game = game;
    }

    //Continuous loop
    public void loop() {
        System.out.println(counter);
        counter++;
        cur_wp = api.getExecutingWayPoint();
        System.out.println("cur wp:: " + cur_wp);
        if (cur_wp == null) { ; }
        else {
            double[] goal_coords = cur_wp.get_waypoint_point(); // get goal coordinates
            double[] my_coords = api.getPosition();             // get cur coordinates
            System.out.println("player thread destination:: " + goal_coords);
            System.out.println("player thread pos:: "+ my_coords);
            SPoint my_point = new SPoint(my_coords[0], my_coords[1], my_coords[2]);
            SPoint goal_point = new SPoint(goal_coords[0], goal_coords[1], goal_coords[2]);
            double dist = my_point.dist(goal_point);

            if (dist <= 0.3) {
                System.out.println("cancelling waypoint, close enough!");
                api.cancelCurrentWayPoint();
            } else {
                System.out.println("player thread:: still going to point");
            }
        }
    }
}