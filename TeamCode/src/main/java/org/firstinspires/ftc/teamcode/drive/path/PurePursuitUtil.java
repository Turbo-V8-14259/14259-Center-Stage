package org.firstinspires.ftc.teamcode.drive.path;


import static java.lang.Math.*;

import org.opencv.core.Point;

import java.util.ArrayList;

public class PurePursuitUtil {
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        double discTolerance = 0.01;
        double m1;
        ArrayList<Point> allPoints = new ArrayList<>();
        // Handle vertical line
        if (linePoint2.x - linePoint1.x != 0) {
            m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        } else {
            allPoints.add(new Point(circleCenter.x, circleCenter.y+radius));
            return allPoints;
        }

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        // Quadratic coefficients of the solution to the intersection of the line segment and circle
        double A = 1 + Math.pow(m1, 2);
        double B = m1 * y1 - Math.pow(m1, 2) * x1;
        double C = Math.pow(y1, 2) - 2 * m1 * y1 * x1 + Math.pow(m1, 2) * Math.pow(x1, 2) - Math.pow(radius, 2);

        double disc = Math.sqrt(Math.pow(B, 2) -  A * C);


        try {
            if (disc > discTolerance) {
                // First root
                double xroot1 = (-B - disc) / ( A);
                double yroot1 = m1 * xroot1;
                xroot1 += circleCenter.x;
                yroot1 += circleCenter.y;

                // Second root
                double xroot2 = (-B + disc) / (A);
                double yroot2 = m1 * xroot2;
                xroot2 += circleCenter.x;
                yroot2 += circleCenter.y;

                if (withinSegment(linePoint1.x, linePoint2.x, xroot1)) {
                    allPoints.add(new Point(xroot1, yroot1));
                }
                if (withinSegment(linePoint1.x, linePoint2.x, xroot2)) {
                    allPoints.add(new Point(xroot2, yroot2));
                }
            } else if (disc >= 0 && disc <= discTolerance) {
                double xroot = -B / A;
                double yroot = m1 * (xroot - x1);
                xroot += circleCenter.x;
                yroot += circleCenter.y;
                allPoints.add(new Point(xroot, yroot));
            }else{
                Point pt = recoveryPt(linePoint1, linePoint2, circleCenter);
                allPoints.add(pt);
            }
        } catch (Exception e) {
            // Handle exceptions
        }

        return allPoints;
    }
    //checks if a point is on a line segment
    public static boolean withinSegment(double start, double end, double value){
        return value>min(start, end)&&value<max(start, end);
    }

    public static Point followMe(Point wayPt1, Point wayPt2, Point robotLocation, double followRadius ){

        ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius,wayPt1, wayPt2);
        if(intersections.size()==2){
            if(abs(wayPt2.x-intersections.get(0).x)<abs(wayPt2.x-intersections.get(1).x)){
                return intersections.get(0);
            }else{
                return intersections.get(1);
            }
        }else if(intersections.size()==1){
            return intersections.get(0);
        }else{
            return robotLocation;//return last robot location?
        }

    }

    public static boolean passedWayPt(Point robotLocation, Point wayPt, Double radius){
        return hypot((robotLocation.y- wayPt.y),(robotLocation.x- wayPt.x)) <= radius;
    }
    public static Point recoveryPt(Point waypt1, Point waypt2, Point robotLocation){
        if(robotLocation.x<min(waypt1.x, waypt2.x)){
            Point newPt = min(waypt1.x, waypt2.x) == waypt1.x ? waypt1:waypt2;
            return newPt;
        }else if(robotLocation.x>max(waypt1.x, waypt2.x)){
            Point newPt = max(waypt1.x, waypt2.x) == waypt1.x ? waypt1:waypt2;
            return newPt;
        }else if(robotLocation.x<=max(waypt1.x, waypt2.x)&&robotLocation.x>=min(waypt1.x, waypt2.x)){
            double m;
            if (waypt2.x - waypt1.x != 0) {
                m = (waypt2.y - waypt1.y) / (waypt2.x - waypt1.x);
                double b = (waypt2.y-waypt1.y) - m*(waypt2.x-waypt1.x);
                double x1 = robotLocation.x;
                double y1 = robotLocation.y - b;
                if(m==0){
                    b = waypt1.y;
                    return new Point(robotLocation.x, b);
                }else{
                    double rootx = (m*(y1+x1*(1/m)))/(pow(m,2)+1);
                    double rooty = m*rootx + b;
                    return new Point(rootx, rooty);
                }
            } else {
                return new Point(waypt1.x, robotLocation.y);
            }
        }else{
            Point pt = hypot(robotLocation.x-waypt1.x, robotLocation.y-waypt1.y) < hypot(robotLocation.x-waypt2.x, robotLocation.y-waypt2.y) ? waypt1 : waypt2;
            return new Point(pt.x, pt.y);
        }
    }
}
