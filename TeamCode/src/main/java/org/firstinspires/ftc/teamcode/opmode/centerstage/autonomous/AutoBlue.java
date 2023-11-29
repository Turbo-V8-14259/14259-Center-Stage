package org.firstinspires.ftc.teamcode.opmode.centerstage.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;
import org.firstinspires.ftc.teamcode.vision.Blue;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.Blue;
import org.firstinspires.ftc.vision.VisionPortal;

import static org.firstinspires.ftc.teamcode.vision.Blue.Location.LEFT;
import static org.firstinspires.ftc.teamcode.vision.Blue.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.vision.Blue.Location.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Blue;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(name = "bad blue autonomy")
public class AutoBlue extends LinearOpMode {

    private Blue.Location location = Blue.Location.MIDDLE;
    private Blue bluePropProcessor;
    private VisionPortal visionPortal;


    int randomization = 2;
    //0 means left, 1 means middle, 2 means right.
    SampleMecanumDrive drive;

    Intake intake;

    LM1Turret turret;
    DepoArm arm;
    DepoSlides slides;
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

        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(90)); //default position
        Pose2d initialLine = new Pose2d(-35, 15, Math.toRadians(90));
        Pose2d leftProp = new Pose2d(-42, 32, Math.toRadians(90));
        Pose2d leftPropIntermediate = new Pose2d(-42, 5, Math.toRadians(90));
        Pose2d stackPos = new Pose2d(-45, 17, Math.toRadians(180));
        Pose2d toStack = new Pose2d(-52, 17, Math.toRadians(180));
        Pose2d toStackMore = new Pose2d(-53, 18, Math.toRadians(180));


        Pose2d runToBoardPos = new Pose2d(45, 18, Math.toRadians(180));
        Pose2d middleStrafe = new Pose2d(59, 41, Math.toRadians(180));
        Pose2d rightProp = new Pose2d(-32, 40, Math.toRadians(0)); //rn
        Pose2d rightPropIntermediate = new Pose2d(-28, 40, Math.toRadians(0)); //rn
        Pose2d rightPropIntermediate2 = new Pose2d(-40, 40, Math.toRadians(0)); //rn

        Pose2d middleProp = new Pose2d(-30, 21, Math.toRadians(90));
        Pose2d middlePropIntermediate = new Pose2d(-35,5, Math.toRadians(90));
        Pose2d leftStrafe = new Pose2d(59, 36, Math.toRadians(180));

        Pose2d rightStrafe = new Pose2d(59, 47, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory beforePropID = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(initialLine)
                .build(); //FIRST LINE (UNIVERSAL)

        Trajectory toPropMiddle = drive.trajectoryBuilder(beforePropID.end())
                .lineToLinearHeading(middleProp)
                .build(); //FIRST PROP  (MIDDLE)

        Trajectory middleIntermediate = drive.trajectoryBuilder(toPropMiddle.end())
                .lineToLinearHeading(middlePropIntermediate)
                .build(); //MIDDLE INTERMEDIATE

        Trajectory middleToStackPosition = drive.trajectoryBuilder(middleIntermediate.end())
                .lineToLinearHeading(stackPos)
                .build(); //MIDDLE GOES TO STACK

        Trajectory middleForwardToStack = drive.trajectoryBuilder(middleToStackPosition.end())
                .lineToLinearHeading(toStack)
                .build(); //MIDDLE FORWARD TO STACK

        Trajectory middleRunToBoardish = drive.trajectoryBuilder(middleForwardToStack.end())
                .lineToLinearHeading(runToBoardPos)
                .build(); //MIDDLE TO BOARD(ISH)

        Trajectory middleStrafeBoard = drive.trajectoryBuilder(middleRunToBoardish.end())
                .lineToLinearHeading(middleStrafe)
                .build(); //MIDDLE TO MIDDLE OF BOARD



        //LEFT SHIT
        Trajectory toPropLeft = drive.trajectoryBuilder(beforePropID.end())
                .lineToLinearHeading(leftProp)
                .build();

        Trajectory toPropLeftInBetween = drive.trajectoryBuilder(toPropLeft.end())
                .lineToLinearHeading(leftPropIntermediate)
                .build();

        Trajectory leftToStackPosition = drive.trajectoryBuilder(toPropLeftInBetween.end())
                .lineToLinearHeading(stackPos)
                .build();

        Trajectory leftForwardToStack = drive.trajectoryBuilder(leftToStackPosition.end())
                .lineToLinearHeading(toStackMore)
                .build();

        Trajectory leftRunToBoardish = drive.trajectoryBuilder(leftForwardToStack.end())
                .lineToLinearHeading(runToBoardPos)
                .build();

        Trajectory leftStrafeBoard = drive.trajectoryBuilder(leftRunToBoardish.end())
                .lineToLinearHeading(leftStrafe)
                .build();


        //RIGHT SIDE

        Trajectory toPropRight = drive.trajectoryBuilder(beforePropID.end())
                .lineToLinearHeading(rightProp)
                .build();

        Trajectory rightIntermediate = drive.trajectoryBuilder(toPropRight.end())
                .lineToLinearHeading(rightPropIntermediate)
                .build();

        Trajectory rightPropIntermediateTwo = drive.trajectoryBuilder(rightIntermediate.end())
                .lineToLinearHeading(rightPropIntermediate2)
                .build();

        Trajectory rightToStackPosition = drive.trajectoryBuilder(rightPropIntermediateTwo.end())
                .lineToLinearHeading(stackPos)
                .build();

        Trajectory rightForwardToStack = drive.trajectoryBuilder(rightToStackPosition.end())
                .lineToLinearHeading(toStackMore)
                .build();

        Trajectory rightRunToBoardish = drive.trajectoryBuilder(rightForwardToStack.end())
                .lineToLinearHeading(runToBoardPos)
                .build();

        Trajectory rightStrafeBoard = drive.trajectoryBuilder(rightRunToBoardish.end())
                .lineToLinearHeading(rightStrafe)
                .build();

        bluePropProcessor = new Blue(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), bluePropProcessor);

        while (!isStarted()) {
            location = bluePropProcessor.getLocation();
            if (location==LEFT){
                randomization = 2;
            } else if (location==MIDDLE){
                randomization = 1;
            } else if (location==RIGHT){
                randomization = 0;
            }
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(beforePropID);
            if(randomization == 1){ //MIDDLE
                drive.followTrajectory(toPropMiddle);
                intake.setState(Intake.IntakeState.AUTO_HIGH);
                intake.update();
                sleep(500);
                drive.followTrajectory(middleIntermediate);
                drive.followTrajectory(middleToStackPosition);
                drive.followTrajectory(middleForwardToStack);

                intake.setPower(-.5);
                intake.setState(Intake.IntakeState.AUTO_STACK_DROPPED);
                intake.update();

                sleep(500);

                drive.followTrajectory(middleRunToBoardish);
                drive.followTrajectory(middleStrafeBoard);

                score();

                intake.setPower(0);
                intake.update();

                sleep(500000);

            }else if(randomization == 0){ //LEFT
                drive.followTrajectory(toPropLeft);
                intake.setState(Intake.IntakeState.AUTO_HIGH);
                intake.update();
                sleep(500);
                drive.followTrajectory(toPropLeftInBetween);
                drive.followTrajectory(leftToStackPosition);
                drive.followTrajectory(leftForwardToStack);

                intake.setPower(-.5);
                intake.setState(Intake.IntakeState.AUTO_STACK_DROPPED);
                intake.update();

                sleep(500);

                drive.followTrajectory(leftRunToBoardish);
                drive.followTrajectory(leftStrafeBoard);

                score();

                intake.setPower(0);
                intake.update();

                sleep(500000);
            }else if(randomization == 2){ //RIGHT
                drive.followTrajectory(toPropRight);
                intake.setState(Intake.IntakeState.AUTO_HIGH);
                intake.update();
                sleep(500);
                drive.followTrajectory(rightIntermediate);
                sleep(500);
                drive.followTrajectory(rightPropIntermediateTwo);
                drive.followTrajectory(rightToStackPosition);
                drive.followTrajectory(rightForwardToStack);

                intake.setPower(-.5);
                intake.setState(Intake.IntakeState.AUTO_STACK_DROPPED);
                intake.update();

                sleep(500);

                drive.followTrajectory(rightRunToBoardish);
                drive.followTrajectory(rightStrafeBoard);

                score();

                intake.setPower(0);
                intake.update();

                sleep(500000);

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
