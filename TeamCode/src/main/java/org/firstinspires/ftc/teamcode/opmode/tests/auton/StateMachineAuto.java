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

@Autonomous(name = "2+5 Test Red")

public class StateMachineAuto extends OpMode {

    enum State {
        IDLE,
        TOINTAKE,
        TOGPL,
        TOSCOREL,
        INTAKE,
        SCORE,
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

    Trajectory leftToIntake;
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
                .build();
        ScoreLeft = drive.trajectoryBuilder(pickUpStack.end())
                .lineToLinearHeading(runToBoardPos)
                .build();
        leftToIntake = drive.trajectoryBuilder(ScoreLeft.end())
                .lineToLinearHeading(toStackMore)
                .build();
        currentstate = State.TOGPL;
        drive.followTrajectoryAsync(toLeftProp);
    }

    @Override

    public void loop() {
        switch (currentstate) {
            case TOGPL:
                if (!drive.isBusy()) {
                    intake.setState(Intake.IntakeState.AUTO_HIGH);
                    if (timeToggle) {
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
                if (!drive.isBusy() && atTargetPosition(toStack)) {
                    currentstate = State.INTAKE;
                }
            case INTAKE:
                if(!drive.isBusy() && atTargetPosition(toStack)) {
                    intake.setPower(-0.5);
                    drive.followTrajectoryAsync(pickUpStack);
                    if (timeToggle) {
                        timeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if (timer.milliseconds() > timeStamp + 800) {
                        drive.followTrajectoryAsync(ScoreLeft);
                        currentstate = State.TOSCOREL;
                        timeToggle = true;
                    }
                }
            case TOSCOREL:
                if(drive.getPoseEstimate().getX() > 5){
                    slides.setState(DepoSlides.DepositState.RUNTOPOSITION);
                    turret.setState(LM1Turret.TurretState.RUNTOPOSITION);
                    arm.setState(DepoArm.DepoArmState.RUNTOPOSITION);
                    arm.manualPosition = 0.4;
                    slides.manualPosition = 6;
                    if (timeToggle) {
                        timeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if (timer.milliseconds() > timeStamp + 600) {
                        turret.manualPosition = 0.6;
                        timeToggle = true;
                    }
                    if(!drive.isBusy()){
                        currentstate = State.SCORE;
                    }
                }
            case SCORE:
                if(!slides.isBusy()){
                    arm.manualPosition = 0.3;
                }
                if (timeToggle) {
                    timeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if (timer.milliseconds() > timeStamp + 600) {
                    turret.manualPosition = 0.6;
                    timeToggle = true;
                }
        }

        slides.update();
        arm.update();
        turret.update();
        intake.update();
        drive.update();
        telemetry.addData("Robot Angle", drive.getPoseEstimate().getHeading());
        telemetry.addData("state: ", currentstate);
    }

    public boolean atTargetPosition(Pose2d targetPose){
        double deltaX = targetPose.getX() - drive.getPoseEstimate().getX();
        double deltaY = targetPose.getY() - drive.getPoseEstimate().getY();
        double deltaR = targetPose.getHeading() - drive.getPoseEstimate().getHeading();
        telemetry.addData("delta Sum", Math.abs(deltaX) + Math.abs(deltaY) + Math.abs(deltaR));
        return deltaX + deltaY + deltaR < 10;
    }
}



