package org.firstinspires.ftc.teamcode.opmode.path;

import static org.firstinspires.ftc.teamcode.opmode.path.PurePursuitUtil.followMe;
import static org.firstinspires.ftc.teamcode.opmode.path.PurePursuitUtil.passedWayPt;

import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotController {
    private Robot robot;

    public RobotController(Robot robot){
        this.robot = robot;
    }
    public void goTo(Point waypt1, Point waypt2, double followRadius, int index){
        while(!passedWayPt(robot.getLocation(), waypt2, followRadius)){
            Point follow = followMe(waypt1, waypt2, robot.getLocation(), followRadius);
            robot.setPosition(follow);
            robot.updateControl();
        }
        System.out.println("Robot passed waypoint "+ index+1);
    }
}
