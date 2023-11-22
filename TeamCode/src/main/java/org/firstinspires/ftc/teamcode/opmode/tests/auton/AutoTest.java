package org.firstinspires.ftc.teamcode.opmode.tests.auton;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Math.CalculateTangents;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;
@Autonomous(name = "async auton")
public class AutoTest extends OpMode {
    enum State {
        IDLE,
        INTAKE,
        TRAVEL,
        SCORE
    }

    SampleMecanumDrive drive;

    Intake intake;

    LM1Turret turret;
    DepoArm arm;
    DepoSlides slides;
    State currentState = State.INTAKE;
    Trajectory trajectory1;
    Trajectory trajectory2;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "Intake"), new ServoMotorBetter(hardwareMap.get(Servo.class, "intakeArm")));
        intake.setState(Intake.IntakeState.INITIALIZE);
        intake.update();

        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));

        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));

        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));

        slides.passive = false;
        slides.pidRunning = true;
        slides.manualMode = false;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90));
        Vector2d middlePose = new Vector2d(0, 20);
        Vector2d endPose = new Vector2d(10, 20);

        trajectory1 = drive.trajectoryBuilder(startPose)
                .lineTo(middlePose)
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajectory2))
                .build();

        trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineTo(endPose, CalculateTangents.calculateTangent(middlePose, endPose))
                .build();

        drive.followTrajectoryAsync(trajectory1);

    }

    @Override
    public void loop() {
        drive.update();


        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addData("Trajectory Busy", drive.isBusy());
        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("State", currentState);
        telemetry.update();


    }

//    public void score() {
//        arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
//        arm.update();
//        sleep(700);
//        turret.setState(LM1Turret.TurretState.SCORE);
//        turret.update();
//        sleep(700);
//        arm.setState(DepoArm.DepoArmState.SCORE);
//        arm.update();
//        sleep(700);
//        arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
//        arm.update();
//        sleep(700);
//        turret.setState(LM1Turret.TurretState.INITIALIZE);
//        turret.update();
//        sleep(700);
//        arm.setState(DepoArm.DepoArmState.INITIALIZE);
//        arm.update();
//    }
}

//        switch(currentState){
//        case IDLE:
//        break;
//
//        case INTAKE:
//        telemetry.addData("task", "trajectory1 is running");
//        telemetry.update();
//        drive.followTrajectoryAsync(trajectory1);
//        currentState = State.TRAVEL;
//        break;
//        case TRAVEL:
//        if(!drive.isBusy()){
//        intake.setPower(-.5);
//        intake.setState(Intake.IntakeState.AUTO_STACK_DROPPED);
//        intake.update();
//        telemetry.addData("task", "trajectory2 is running");
//        telemetry.update();
//        drive.followTrajectoryAsync(trajectory2);
//
//        }
//        currentState = State.SCORE;
//        break;
//        case SCORE:
//        score();
//        intake.setPower(0);
//        intake.update();
//        currentState = State.IDLE;
//        break;
//        }