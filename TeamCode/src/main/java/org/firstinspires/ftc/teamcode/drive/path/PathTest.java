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

@TeleOp (name = "PPTest")
@Config
public class PathTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    double previousTime = 0 ;
    double currentTime = 0;

    int currentWaypointIndex = 0;
    double lookaheadRadius = 20;
    double lookaheadRadiusHeading = 23;
    double updateTime;

    DT drive;

    int i = 0;
    @Override
    public void runOpMode() throws InterruptedException {


        drive = new DT(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.followRadius = lookaheadRadius;


        waitForStart();
        while(opModeIsActive()) {
            currentTime = timer.milliseconds()/(Math.pow(10,9));
            if(i == 0){
                ArrayList<Pose2d> wayPoints = new ArrayList<>();
                wayPoints.add(new Pose2d(0, 0));
                wayPoints.add(new Pose2d(10,30));
                wayPoints.add(new Pose2d(57,30));
                wayPoints.add(new Pose2d(57,0));
                if(Math.hypot((57 - drive.getX()), 0 - drive.getY()) < 20){
                    drive.setPathEndHold(true);
                    drive.lineTo(57,0, Math.toRadians(-90));
                    if(Math.hypot((57 - drive.getX()), 0 - drive.getY()) < 3) {
                        i++;
                    }
                }else{
                    followPath(wayPoints, .2, 23, 20, 0, -99, drive);
                }
            }else if(i==1){
                drive.setPathEndHold(false);
                ArrayList<Pose2d> wayPoints1 = new ArrayList<>();
                wayPoints1.add(new Pose2d(57,0));
                wayPoints1.add(new Pose2d(0, 0));
                followPath(wayPoints1, .2, 23, 20, 0, -99, drive);
            }
//            wayPoints.add(new Pose2d(72,10));
            telemetry.addData("i ", i);
            telemetry.addData("hz ", 1/(currentTime-previousTime));
            telemetry.addData("delta from final wp1 ", Math.hypot((80 - drive.getX()), 0 - drive.getY()));
            telemetry.update();
            drive.update();
            previousTime = currentTime;
        }


    }

    Pose2d lastTranslatePoint = new Pose2d(0,0);
    Pose2d lastHeadingPoint = new Pose2d(0,0);
    void followPath(ArrayList<Pose2d> path, double movePower, double headingRadius, double moveRadius, double headingOffset, double lockAngle, DT drive) {
        Pose2d followDrive = PurePursuitUtil.followMe(path, drive.getLocation(), moveRadius, lastTranslatePoint, false);
        lastTranslatePoint = followDrive;

        Pose2d followHeading = PurePursuitUtil.followMe(path, drive.getLocation(), headingRadius, lastHeadingPoint, true);
        lastHeadingPoint = followHeading;

        drive.lineTo(followDrive.getX(), followDrive.getY(),drive.toPoint(drive.getX(), drive.getY(), drive.getR(), followHeading.getX(), followHeading.getY() + headingOffset));

        drive.setMaxPower(movePower);
    }


}