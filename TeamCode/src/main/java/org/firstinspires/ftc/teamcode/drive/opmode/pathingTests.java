package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled

public class pathingTests extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d initialPosition = new Pose2d(59, -38, Math.toRadians(0));
        Vector2d secondPosition = new Vector2d(12,-38);
        Pose2d thirdPosition = new Pose2d(35,-32,Math.toRadians(90));
        Pose2d fourthPosition = new Pose2d(12,-50,Math.toRadians(-90));
        Pose2d fifthPosition = new Pose2d(12,-55, Math.toRadians(-90));
        Pose2d sixthPosition = new Pose2d(12,0, Math.toRadians(-90));
        Pose2d seventhPosition = new Pose2d(15,11, Math.toRadians(-120));
        drive.setPoseEstimate(initialPosition);
        waitForStart();
        if (isStopRequested()) return;

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d(59, -35, Math.toRadians(0)))
//          Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(11,-35,Math.toRadians(0)))
////                .lineToLinearHeading(new Pose2d(35,-23,Math.toRadians(45))) //or y -47 and -45degree for down randomization and x 24 for straight
////                .lineToLinearHeading(new Pose2d(11,-35,Math.toRadians(-90)))
////                .lineToLinearHeading(new Pose2d(11,-55, Math.toRadians(-90)))
////                .lineToLinearHeading(new Pose2d(11,11, Math.toRadians(-90)))
//                .build();

        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(35,-23,Math.toRadians(45))) //or y -47 and -45degree for down randomization and x 24 for straight
                .lineTo(secondPosition)
                .splineToSplineHeading(thirdPosition, calculateTangent(secondPosition,thirdPosition))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(fourthPosition)
                .splineToSplineHeading(fifthPosition, calculateTangent(fourthPosition,fifthPosition))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(sixthPosition)
                .splineToSplineHeading(seventhPosition, calculateTangent(sixthPosition,seventhPosition))
                .build();
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .lineToLinearHeading(new Pose2d(11,11, Math.toRadians(-90)))
//                .build();
//                 .lineTo(secondPosition)
//                .splineToLinearHeading(thirdPosition, calculateTangent(secondPosition,thirdPosition)) //or y -47 and -45degree for down randomization and x 24 for straight
//                .lineToLinearHeading(fourthPosition)
//                .splineToLinearHeading(fifthPosition, calculateTangent(fourthPosition,fifthPosition))
////                .lineToLinearHeading(sixthPosition)
//                .splineToSplineHeading(seventhPosition, calculateTangent(sixthPosition,seventhPosition))
//        drive.followTrajectory(traj);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        drive.turn(Math.toRadians(-30));
        drive.update();


    }
    public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Pose2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
}
