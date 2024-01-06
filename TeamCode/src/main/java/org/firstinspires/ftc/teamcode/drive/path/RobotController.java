//package org.firstinspires.ftc.teamcode.drive.path;
//
//import org.opencv.core.Point;
//
//public class RobotController {
//    private Robot robot;
//
//    public RobotController(Robot robot){
//        this.robot = robot;
//    }
//    public void goTo(Point waypt1, Point waypt2, double followRadius, int index){
//        while(!PurePursuitUtil.passedWayPt(robot.getLocation(), waypt2, followRadius)){
//            Point follow = PurePursuitUtil.followMe(waypt1, waypt2, robot.getLocation(), followRadius);
//            robot.setPosition(follow);
//            robot.updateControl();
//        }
//        System.out.println("Robot passed waypoint "+ index+1);
//    }
//}
