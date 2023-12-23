package org.firstinspires.ftc.teamcode.opmode.path;

import static java.lang.Math.hypot;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import org.opencv.core.Point;

import java.util.ArrayList;

public class PurePursuitUtil {
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){

        double r = hypot(circleCenter.x, circleCenter.y);
        double m1 = (linePoint2.y-linePoint1.y)-(linePoint2.x-linePoint1.y);
        double x1 = linePoint1.x-circleCenter.x;
        double y1 = linePoint1.y-linePoint1.y;

        double A = 1 + pow(m1, 2);
        double B = (2*m1*y1) -(2*pow(m1,2)*x1);
        double C = (pow(y1, 2)) - (2*m1*y1*x1) -(pow(m1, 2)*pow(x1, 2)) - (pow(r, 2));

        double root1 = ((-2*B)-sqrt(pow(B,2)-4*A*C))/(2*A);
        double root2 = ((-2*B)+sqrt(pow(B,2)-4*A*C))/(2*A);

    }
}
