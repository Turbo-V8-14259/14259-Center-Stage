package org.firstinspires.ftc.teamcode.drive.path;

import org.opencv.core.Point;

import java.util.ArrayList;

public class PurePursuit {

    public static void main(String[] args) {
        double lookaheadDist = 5.0;
        Robot robot = new Robot(new Point(0, 0));
        RobotController controller = new RobotController(robot);
        ArrayList<Point> wayPoints = new ArrayList<>();
        wayPoints.add(robot.getLocation());
        wayPoints.add(new Point(0, 50));
        wayPoints.add(new Point(23, 70));
        wayPoints.add(new Point(90, 30));
        wayPoints.add(new Point(-80, -60));

        for (int i = 0; i < wayPoints.size() - 1; ++i) {
            controller.goTo(wayPoints.get(i), wayPoints.get(i + 1), lookaheadDist, i);
        }
    }
}
