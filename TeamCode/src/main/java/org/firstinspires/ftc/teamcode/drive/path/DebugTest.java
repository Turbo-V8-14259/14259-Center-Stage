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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.setFollowRadius(lookaheadRadius);
        DebugUtil.updateSegment(1);
        DebugUtil.updateEnding(false);

        waitForStart();
        while(opModeIsActive()) {
//            drive.setPathEndHold(true);
            previousTime = currentTime;
            currentTime = timer.nanoseconds()/1000000000.0;


            ArrayList<Pose2d> wayPoints = new ArrayList<>();
            wayPoints.add(new Pose2d(0, 0));
            wayPoints.add(new Pose2d(0, 60));
            wayPoints.add(new Pose2d(20, 0));
            wayPoints.add(new Pose2d(50, 60));

            wayPoints.add(new Pose2d(70, 30));


            followPath(wayPoints, 1, 13, 13, 0, -99, drive);





            telemetry.addData("i ", i);
            telemetry.addData("hz ", 1/(currentTime-previousTime));
            telemetry.addData("segment ", DebugUtil.getSegment());
            telemetry.update();

            drive.update();
        }


    }

    Pose2d lastTranslatePoint = new Pose2d(0,0);
    Pose2d lastHeadingPoint = new Pose2d(0,0);
    public void followPath(ArrayList<Pose2d> path, double movePower, double headingRadius, double moveRadius, double headingOffset, double lockAngle, DT drive) {

        //false for urm non heading ig
        Pose2d followDrive = DebugUtil.followMe(path, drive.getLocation(), moveRadius, lastTranslatePoint, false);
        lastTranslatePoint = followDrive;

        //true for heading

        Pose2d followHeading = DebugUtil.followMe(path, drive.getLocation(), headingRadius, lastHeadingPoint, true);
        lastHeadingPoint = followHeading;
        if(DebugUtil.getEnding()) {
            drive.setPathEndHold(true);
            drive.lineTo(followDrive.getX(), followDrive.getY(),Math.toRadians(-180));
        }else{
            drive.lineTo(followDrive.getX(), followDrive.getY(),drive.toPoint(drive.getX(), drive.getY(), drive.getR(), followHeading.getX(), followHeading.getY() + headingOffset));
        }


        drive.setMaxPower(movePower);



    }


}