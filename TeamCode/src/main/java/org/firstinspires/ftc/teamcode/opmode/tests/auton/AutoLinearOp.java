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

@Autonomous
public class AutoLinearOp extends LinearOpMode {

    enum State{
        IDLE,
        TEAMPROP,
        INTAKE,
        TRAVEL,
        STACK, //picks up pixels from stack
        BOARD, //to scoring board
        SCORE //arm scores
    }
    private Red.Location location = Red.Location.MIDDLE;
    private Red redPropProcessor;
    private VisionPortal visionPortal;
    int randomization = 2;
    //0 means left, 1 means middle, 2 means right.
    SampleMecanumDrive drive;

    Intake intake;

    LM1Turret turret;
    DepoArm arm;
    DepoSlides slides;

    State currentstate = State.IDLE;

    Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(-90));
    Pose2d initialLine = new Pose2d(-35, -15, Math.toRadians(-90));

    Pose2d leftProp = new Pose2d(-42, -32, Math.toRadians(-90));
    Pose2d rightProp = new Pose2d(-32, -40, Math.toRadians(0));
    Pose2d middleProp = new Pose2d(-30, -21, Math.toRadians(-90));
    Pose2d prop[] = {leftProp, middleProp, rightProp};
    Pose2d toStack = new Pose2d(-52, -17, Math.toRadians(-180));
    Pose2d middleStrafe = new Pose2d(59, -41, Math.toRadians(-180));
    Pose2d leftStrafe = new Pose2d(59, -36, Math.toRadians(-180));

    Pose2d rightStrafe = new Pose2d(59, -47, Math.toRadians(-180));

    Pose2d board[] = {leftStrafe, middleStrafe, rightStrafe};

    Pose2d runToBoardPos = new Pose2d(45, -18, Math.toRadians(-180));

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

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(initialLine)
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(prop[randomization])
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(initialLine)
                .lineToLinearHeading(toStack)
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .lineToLinearHeading(runToBoardPos)
                .lineToLinearHeading(board[randomization])
                .build();
        redPropProcessor = new Red(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor);

        while(!isStarted()){
            location = redPropProcessor.getLocation();
            if(location==LEFT){
                randomization = 0;
            }else if(location==MIDDLE){
                randomization = 1;
            }else if(location==RIGHT){
                randomization = 2;
            }
            telemetry.update();
        }
        currentstate = State.TRAVEL;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()){
            switch(currentstate){
                case TRAVEL:
                    if(!drive.isBusy()){
                        currentstate = State.TEAMPROP;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                case TEAMPROP:
                    if(!drive.isBusy()) {
                        currentstate = State.STACK;
                        intake.setState(Intake.IntakeState.AUTO_HIGH);
                        intake.update();
                        sleep(500);
                        drive.followTrajectory(trajectory3);
                    }
                case STACK:
                    if(!drive.isBusy()){
                        currentstate = State.BOARD;
                        intake.setPower(-.5);
                        intake.setState(Intake.IntakeState.AUTO_STACK_DROPPED);
                        intake.update();

                        sleep(500);
                        drive.followTrajectory(trajectory4);
                    }
                case BOARD:
                    if(!drive.isBusy()){
                        score();
                    }
            }
        }


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
