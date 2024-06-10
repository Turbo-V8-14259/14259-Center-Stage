package org.firstinspires.ftc.teamcode.drive.path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.path.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;

@TeleOp (name = "Debug PP")
@Config
public class DebugTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    double previousTime = 0;
    double currentTime = 0;

    double lookaheadRadius = 20;

    DT drive;

    int i = 0;
    @Override
    public void runOpMode() throws InterruptedException {


        drive = new DT(hardwareMap);
        drive.setPathEndHold(false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.setFollowRadius(lookaheadRadius);
        DebugUtil.updateSegment(1);

        waitForStart();
        while(opModeIsActive()) {
            previousTime = currentTime;
            currentTime = timer.nanoseconds()/10000000000.0;
            if(i == 0){
                ArrayList<Pose2d> wayPoints = new ArrayList<>();
                wayPoints.add(new Pose2d(0, 0));
                wayPoints.add(new Pose2d(35,70));
                wayPoints.add(new Pose2d(70, 0));
               
                wayPoints.add(new Pose2d(0, 0));




                followPath(wayPoints, .5, 23, 20, 0, -99, drive);
                //this logic doenst work?
//                followPath(wayPoints, .5, 23, 20, 0, -99, drive);
//                if(Math.hypot((57 - drive.getX()), 50 - drive.getY()) < 20) {
//                    i++;
//                }
            }
//            wayPoints.add(new Pose2d(72,10));
            telemetry.addData("i ", i);
            telemetry.addData("hz ", 1/(currentTime-previousTime));
            telemetry.addData("delta from final wp1 ", Math.hypot((80 - drive.getX()), 0 - drive.getY()));
            telemetry.addData("robot x", drive.getX());
            telemetry.addData("robot y", drive.getY());
            telemetry.addData("segment", DebugUtil.getSegment());
            telemetry.update();
            drive.update();
        }


    }

    Pose2d lastTranslatePoint = new Pose2d(0,0);
    Pose2d lastHeadingPoint = new Pose2d(0,0);
    void followPath(ArrayList<Pose2d> path, double movePower, double headingRadius, double moveRadius, double headingOffset, double lockAngle, DT drive) {
        Pose2d followDrive = DebugUtil.followMe(path, drive.getLocation(), moveRadius,  lastTranslatePoint);
        lastTranslatePoint = followDrive;


        Pose2d followHeading = DebugUtil.followMe(path, drive.getLocation(), moveRadius,  lastHeadingPoint);
        lastHeadingPoint = followHeading;

        drive.lineTo(followDrive.getX(), followDrive.getY(),drive.toPoint(drive.getX(), drive.getY(), drive.getR(), followHeading.getX(), followHeading.getY() + headingOffset));

        drive.setMaxPower(movePower);

    }


}