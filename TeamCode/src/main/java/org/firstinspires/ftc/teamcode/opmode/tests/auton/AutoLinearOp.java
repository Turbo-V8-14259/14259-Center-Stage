package org.firstinspires.ftc.teamcode.opmode.tests.auton;

import static org.firstinspires.ftc.teamcode.vision.Red.Location.LEFT;
import static org.firstinspires.ftc.teamcode.vision.Red.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.vision.Red.Location.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;
import org.firstinspires.ftc.teamcode.vision.Red;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.teamcode.usefuls.Math.CalculateTangents;

@Autonomous(name = "no work welp")

public class AutoLinearOp extends OpMode {

    enum State {
        IDLE,
        TOINTAKE,
        GROUNDPL,
        PRELOAD,
        TEAMPROP,
        INTAKE,
        SCORE,
        TRAVEL,
        STACK, //picks up pixels from stack
        BOARD, //to scoring board
    }

    int randomization = 0;
    //0 means left, 1 means middle, 2 means right.
    SampleMecanumDrive drive;
    boolean timeToggle = true;
    double timeStamp = 0;
    Intake intake;

    ElapsedTime timer = new ElapsedTime();
    LM1Turret turret;
    DepoArm arm;
    DepoSlides slides;

    State currentstate;


    //First Diverge

    Pose2d startPose = new Pose2d(-35, -65, Math.toRadians(-90));
    Pose2d leftProp = new Pose2d(-42, -32, Math.toRadians(-90));
    Pose2d leftPropIP = new Pose2d(-42, -17, Math.toRadians(-90));
    Pose2d toStack = new Pose2d(-52, -17, Math.toRadians(-180));
    Pose2d toStackMore = new Pose2d(-53, -18, Math.toRadians(-180));
    Pose2d runToBoardPos = new Pose2d(45, -18, Math.toRadians(-180));

    Trajectory toLeftStack;
    Trajectory LeftBackFiveInches;
    Trajectory toLeftProp;
    Trajectory pickUpStack;
    Trajectory ScoreLeft;


    // Test Commit

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

        drive.setPoseEstimate(startPose);

        toLeftProp = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(leftProp)
                .build();

        LeftBackFiveInches = drive.trajectoryBuilder(toLeftProp.end())
                .lineToLinearHeading(leftPropIP)
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toLeftStack))
                .build();
        toLeftStack = drive.trajectoryBuilder(LeftBackFiveInches.end())
                .splineToLinearHeading(toStack, CalculateTangents.calculateTangent(leftPropIP, toStack))
                .build();
        pickUpStack = drive.trajectoryBuilder(toLeftStack.end())
                .lineToLinearHeading(toStackMore)
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(ScoreLeft))
                .build();
        ScoreLeft = drive.trajectoryBuilder(pickUpStack.end())
                .lineToLinearHeading(runToBoardPos)
                .build();


        currentstate = State.GROUNDPL;


        drive.followTrajectoryAsync(toLeftProp);
    }

    @Override

    public void loop() {
        switch (currentstate) {
            case GROUNDPL:
                if (!drive.isBusy()) {
                    intake.setState(Intake.IntakeState.AUTO_HIGH);

                    if (timeToggle) {//timeToggle starts at true by default
                        timeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if (timer.milliseconds() > timeStamp + 500) {
                        currentstate = State.TOINTAKE;
                        drive.followTrajectoryAsync(LeftBackFiveInches);
                        timeToggle = true;
                    }
                }
            case TOINTAKE:
                if (timeToggle) {//timeToggle starts at true by default
                    timeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if (timer.milliseconds() > timeStamp + 500) {
                    timeToggle = true;
                    if (!drive.isBusy()) {
                        currentstate = State.INTAKE;
                    }
                }
            case INTAKE:
                if(!drive.isBusy()) {
                    currentstate = State.SCORE;
                    intake.setPower(-0.5);
                    drive.followTrajectoryAsync(pickUpStack);
                }
        }


        intake.update();
        drive.update();
        telemetry.addData("Robot Angle", drive.getPoseEstimate().getHeading());
        telemetry.addData("state: ", currentstate);
    }
}


//    public void score(){
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

