package org.firstinspires.ftc.teamcode.opmode.tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;
import org.firstinspires.ftc.teamcode.vision.CameraPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.usefuls.Math.CalculateTangents;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;


@Autonomous
public class AutoTest extends LinearOpMode {

    enum State{
        IDLE,
        BEFOREPROPID,
        PROPID,
        YELLOWPIXEL,
        INTAKE,
        DEPOSITYELLOW,
        STACK,
        DEPOSITNORMAL

    }

    SampleMecanumDrive drive;
    CameraPipeline cameraPipeline;
    State currentState = State.IDLE;

    Intake intake;

    LM1Turret turret;
    DepoArm arm;
    Pitch pitch;
    int scoringState;
    boolean timeToggle;
    ElapsedTime timer;
    double TimeStamp;
    DepoSlides slides;
    boolean afterYellow; //initialised to false
    int propID;
    @Override
    public void runOpMode() throws InterruptedException{
        drive = new SampleMecanumDrive(hardwareMap);
        cameraPipeline = new CameraPipeline(telemetry, "");
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "Intake"), new ServoMotorBetter(hardwareMap.get(Servo.class, "intakeArm")));
        intake.setState(Intake.IntakeState.INITIALIZE);
        intake.update();

        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));

        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));

        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));

        timer = new ElapsedTime();
        slides.passive = false;
        slides.pidRunning = true;
        slides.manualMode = false;
        
        propID = 0;//a number between 0 and 2 that identifies which direction the prop is in

        Pose2d startPose = new Pose2d(-35, -65, Math.toRadians(-90)); //default position

        Pose2d leftProp = new Pose2d(-35, -29, Math.toRadians(-180));
        Pose2d rightProp = new Pose2d(-35, -29, Math.toRadians(0));
        Pose2d middleProp = new Pose2d(-35, -16, Math.toRadians(-90));
        Pose2d propDir[] = {leftProp, middleProp, rightProp};
        Pose2d afterPropID = new Pose2d(-35, -11.6, Math.toRadians(-90));
        Vector2d beforeYellow = new Vector2d(25, -11.6);
        double depoAngle[] = {Math.toRadians(-165),  Math.toRadians(-180),Math.toRadians(-195) };
        Pose2d yellowPixel = new Pose2d(35, -34, depoAngle[propID]);
        Pose2d stackIntermdiate = new Pose2d(25, -11.6, Math.toRadians(180));
        Vector2d stack = new Vector2d(-55, -11.6);

        drive.setPoseEstimate(startPose); //used when start isnt default (0, 0, 0)

        Trajectory beforePropID = drive.trajectoryBuilder(startPose)
                .back(65-11.6)
                .build();

        Trajectory toProp = drive.trajectoryBuilder(beforePropID.end())
                .splineToSplineHeading(propDir[propID], CalculateTangents.calculateTangent(new Vector2d(-35, -11.6), propDir[propID]))
                .build();

        Trajectory depositYellow = drive.trajectoryBuilder(toProp.end())
                .splineToSplineHeading(afterPropID, CalculateTangents.calculateTangent(propDir[propID], afterPropID))
                .lineTo(beforeYellow)
                .splineToSplineHeading(yellowPixel, CalculateTangents.calculateTangent(beforeYellow, yellowPixel))
                .build();

        Trajectory toStack = drive.trajectoryBuilder(depositYellow.end())
                .splineToSplineHeading(stackIntermdiate, CalculateTangents.calculateTangent(stackIntermdiate, yellowPixel))
                .lineTo(stack)
                .build();

        Trajectory score = drive.trajectoryBuilder(toStack.end())
                .lineTo(beforeYellow)
                .splineToSplineHeading(yellowPixel, CalculateTangents.calculateTangent(beforeYellow, yellowPixel))
                .build();
        Trajectory toStack2 = drive.trajectoryBuilder(score.end())
                .splineToSplineHeading(stackIntermdiate, CalculateTangents.calculateTangent(stackIntermdiate, yellowPixel))
                .lineTo(stack)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while(opModeInInit()){
            String objectDirection = cameraPipeline.getObjectDirection();
            updatePropID(objectDirection);
        }

        currentState = State.BEFOREPROPID;
        drive.followTrajectoryAsync(beforePropID);

        while(opModeIsActive()&&!isStopRequested()){
            switch(currentState){
                case BEFOREPROPID:
                    if (!drive.isBusy()) {
                        currentState = State.PROPID;
                        drive.followTrajectoryAsync(toProp);
                    }
                    break;
                case PROPID:
                    if(!drive.isBusy()){
                        currentState = State.YELLOWPIXEL;
                    }
                    break;
                case YELLOWPIXEL:
                    if(!drive.isBusy()){
                        currentState = State.DEPOSITYELLOW;
                        drive.followTrajectoryAsync(depositYellow);
                    }
                    break;
                case DEPOSITYELLOW:
                    if(!drive.isBusy()){
                        currentState = State.STACK;
                        scoring();
                    }
                    break;
                case STACK:
                    if(!drive.isBusy()){
                        currentState = State.DEPOSITNORMAL;
                        if(afterYellow){
                            drive.followTrajectoryAsync(toStack2);
                        }else{
                            drive.followTrajectoryAsync(toStack);
                            afterYellow = true;
                        }
                    }
                    break;
                case DEPOSITNORMAL:
                    if(!drive.isBusy()){
                        currentState = State.STACK;
                        drive.followTrajectoryAsync(score);
                        scoring();
                    }
                    break;
            }

            drive.update();
        }


    }
    public void scoring(){
        switch (scoringState){
            case 0: //Init
                slides.setState(DepoSlides.DepositState.DOWN);
                pitch.setState(Pitch.PitchState.INITIALIZE);
                arm.setState(DepoArm.DepoArmState.TRANSFER);
                break;
            case 1: //arm / pitch / slides up
                pitch.setState(Pitch.PitchState.SCOREATLEVEL);
                arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);

                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()> TimeStamp + 500){
                    scoringState=2;
                    timeToggle = true;
                }
                break;
            case 2: // turret move
                slides.setState(DepoSlides.DepositState.CALCULATED_UP);
                turret.setState(LM1Turret.TurretState.SCORE);
                if(timeToggle){//timeToggle starts at true by default
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()> TimeStamp + 300){
                    arm.setState(DepoArm.DepoArmState.INTERMEDIATE);
                    timeToggle = true;
                }

                break;
            case 3:
                arm.setState(DepoArm.DepoArmState.SCORE);
                break;
            case 4: // raising ARM
                arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                if(timeToggle){
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()>TimeStamp + 100){
                    scoringState=5;
                    timeToggle=true;
                }
                break;
            case 5: //resetting turret
                turret.setState(LM1Turret.TurretState.INITIALIZE);
                slides.setState(DepoSlides.DepositState.DOWN);
                if(slides.getCurrentPosition() > 0.2){
                    break;
                }
                pitch.setState(Pitch.PitchState.INITIALIZE);
                if(timeToggle){
                    TimeStamp = timer.milliseconds();
                    timeToggle = false;
                }
                if(timer.milliseconds()>TimeStamp + 500){
                    scoringState=0;
                    timeToggle=true;
                }

                break;
            default:
                scoringState = 0;
                break;
        }
    }
    public void updatePropID(String objDir){
        if(objDir == "LEFT"){
            propID = 0;
            telemetry.addData("Direction", "Left");
        }
        else if(objDir =="MIDDLE"){
            propID = 1;
            telemetry.addData("Direction", "Middle");
        }
        else if(objDir == "Right"){
            propID = 2;
            telemetry.addData("Direction", "Right");
        }
        else{
            telemetry.addData("Direction", "Unknown Direction");
        }
    }

}
