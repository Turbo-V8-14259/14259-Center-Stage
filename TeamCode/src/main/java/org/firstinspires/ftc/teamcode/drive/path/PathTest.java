package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.path.PurePursuitUtil;

@TeleOp (name = "PPTest")
@Config

public class PathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int currentWaypointIndex = 0;
        double lookaheadRadius = 5.0;
        DT drive = new DT(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final ArrayList<Pose2d> wayPoints = new ArrayList<>();
        wayPoints.add(new Pose2d(0, 0));
        wayPoints.add(new Pose2d(10,20 ));
        wayPoints.add(new Pose2d(30,15 ));

        waitForStart();
        while(opModeIsActive()){
            if (currentWaypointIndex < wayPoints.size() - 1) {

                Pose2d currentWaypoint = wayPoints.get(currentWaypointIndex);
                Pose2d nextWaypoint = wayPoints.get(currentWaypointIndex + 1);
                while(!PurePursuitUtil.passedWayPt(drive.getLocation(), nextWaypoint, lookaheadRadius)){
                    Pose2d follow = PurePursuitUtil.followMe(currentWaypoint, nextWaypoint, drive.getLocation(), lookaheadRadius);
                    drive.lineTo(follow.getX(),follow.getY(), follow.getHeading());
                    drive.update();
                    telemetry.update();
                }
                telemetry.addData("Robot passed waypoint ", (currentWaypointIndex+1));
                currentWaypointIndex++;

            } else {
                telemetry.addData("Robot reached the final waypoint.", 0);
            }


        }
    }
}