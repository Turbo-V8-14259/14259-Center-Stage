package org.firstinspires.ftc.teamcode.opmode.tests.auton;

import static org.firstinspires.ftc.teamcode.vision.Red.Location.LEFT;
import static org.firstinspires.ftc.teamcode.vision.Red.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.vision.Red.Location.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous
public class AutoLinearOp extends LinearOpMode {

    enum State{
        IDLE,
        TEAMPROP,
        INTAKE,
        TRAVEL,
        STACK, //picks up pixels from stack
        BOARD, //to scoring board
    }

    int randomization = 0;
    //0 means left, 1 means middle, 2 means right.
    SampleMecanumDrive drive;

    Intake intake;

    LM1Turret turret;
    DepoArm arm;
    DepoSlides slides;

    State currentstate = State.IDLE;


    //First Diverge

    Pose2d startPose = new Pose2d(-35, -65, Math.toRadians(-90));

    Pose2d leftProp = new Pose2d(-42, -32, Math.toRadians(-90));
    Pose2d toStack = new Pose2d(-52, -17, Math.toRadians(-180));
    Trajectory trajectory1;
    Trajectory trajectory2;

    @Override
    public void runOpMode() throws InterruptedException {

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

        trajectory1 = drive.trajectoryBuilder(startPose)
        .lineToLinearHeading(leftProp)
        .addDisplacementMarker(() -> drive.followTrajectoryAsync(trajectory2))
        .build();

        trajectory2 = drive.trajectoryBuilder(trajectory1.end())

        .splineToLinearHeading(toStack, CalculateTangents.calculateTangent(leftProp, toStack))
        .build();

        currentstate = State.TRAVEL;

        while (opModeIsActive() && !isStopRequested()){
            drive.followTrajectoryAsync(trajectory1);
            /*switch(currentstate){
                case TRAVEL:
                    if(!drive.isBusy()){
<<<<<<< HEAD
                        currentstate = State.TEAMPROP;
=======
                        currentstate = State.IDLE;
                        drive.followTrajectoryAsync(trajectory2);
>>>>>>> 8f7cdf86aa526993d9705bf4d170fa62433c2599
                    }
                case TEAMPROP:
                    if(!drive.isBusy()) {
                        currentstate = State.STACK;

                    }
                case STACK:
                    if(!drive.isBusy()){
                        currentstate = State.BOARD;

                    }
                case BOARD:
                    if(!drive.isBusy()){
                        score();
                    }
            }*/
            drive.update();
            telemetry.addData("Robot Angle", drive.getPoseEstimate().getHeading());
        }


    }

    public void initTraj(){
        //nvm
    }
    public void score(){
        arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
        arm.update();
        sleep(700);
        turret.setState(LM1Turret.TurretState.SCORE);
        turret.update();
        sleep(700);
        arm.setState(DepoArm.DepoArmState.SCORE);
        arm.update();
        sleep(700);
        arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
        arm.update();
        sleep(700);
        turret.setState(LM1Turret.TurretState.INITIALIZE);
        turret.update();
        sleep(700);
        arm.setState(DepoArm.DepoArmState.INITIALIZE);
        arm.update();
    }
}
