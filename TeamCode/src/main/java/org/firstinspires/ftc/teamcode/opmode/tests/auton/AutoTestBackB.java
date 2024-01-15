package org.firstinspires.ftc.teamcode.opmode.tests.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@TeleOp(name = "auto test back blue")
@Config
public class AutoTestBackB extends LinearOpMode {

    enum State {
        PROP,
        STACK,
        DEPOSIT,
        PARK
    }
    int x=0;
    int y=-12;
    //robot parts
    Intake intake;

    int cycles_left = 3;

    LM1Turret turret;
    DepoArm arm;
    DepoSlides slides;
    //start location
    Pose2d startBPose = new Pose2d(-35+x, 65+y, Math.toRadians(90)); //default position
    //props
    Pose2d leftBProp = new Pose2d(-35+x, 32+y, Math.toRadians(180));
    Vector2d leftBPropIntermediate = new Vector2d(-37+x, 32+y);

    Vector2d middleBProp = new Vector2d(-35+x, 12+y);
    Vector2d middleBPropIntermediate = new Vector2d(-35+x,14+y);
    Pose2d rightBProp = new Pose2d(-35+x, 32+y, Math.toRadians(0));
    Vector2d rightBPropIntermediate = new Vector2d(-32+x, 32+y);

    //stack
    Pose2d beforeStack = new Pose2d(-37+x, 12+y, Math.toRadians(180));
    Pose2d stackPos = new Pose2d(-54+x, 12+y, Math.toRadians(180));

    //deposit
    Vector2d dropOff = new Vector2d(32+x, 12+y);

    Pose2d depositL = new Pose2d(35+x, 16+y, Math.toRadians(200));
    Pose2d depositM = new Pose2d(35+x, 16+y, Math.toRadians(210));
    Pose2d depositR = new Pose2d(35+x, 16+y, Math.toRadians(220));
    Pose2d afterDepo = new Pose2d(35+x, 12+y, Math.toRadians(180));


    Vector2d runToBoardPos = new Vector2d(55+x, -12+y);

    State currentState;

    int randomization = 0;
    int intermediaterandomizationstate =0;
    @Override
    public void runOpMode() throws InterruptedException {

        DT drive = new DT(hardwareMap, new Pose2d(startBPose.getX(), startBPose.getY(), startBPose.getHeading()));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //initiations
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "Intake"), new ServoMotorBetter(hardwareMap.get(Servo.class, "intakeArm")));
        intake.setState(Intake.IntakeState.INITIALIZE);
        intake.update();
        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));
        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));
        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));

        slides.passive = false;
        slides.pidRunning = true;
        slides.manualMode = false;

        currentState  = State.PROP;
        waitForStart();
        while(opModeIsActive()){
            switch(currentState){
                case PROP:
                    if(randomization==0){ //left
                        if(intermediaterandomizationstate == 0){
                            drive.lineTo(leftBProp.getX(), leftBProp.getY(), leftBProp.getHeading());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 1){
                            drive.lineToCHeading(leftBPropIntermediate.getX(), leftBPropIntermediate.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 2){
                            drive.lineToCHeading(leftBProp.getX(), leftBProp.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 3){
                            drive.lineToCHeading(beforeStack.getX(), beforeStack.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate = 0;
                                currentState = State.STACK;
                            }
                        }
                    }else if(randomization==1){ // middle
                        if(intermediaterandomizationstate == 0){
                            drive.lineToCHeading(middleBProp.getX(), middleBProp.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 1){
                            drive.lineToCHeading(middleBPropIntermediate.getX(), middleBPropIntermediate.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 2){
                            drive.lineToCHeading(middleBProp.getX(), middleBProp.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate = 0;
                                currentState = State.STACK;
                            }
                        }
                    }else{ //right
                        if(intermediaterandomizationstate == 0){
                            drive.lineTo(rightBProp.getX(), rightBProp.getY(), rightBProp.getHeading());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 1){
                            drive.lineToCHeading(rightBPropIntermediate.getX(), rightBPropIntermediate.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 2){
                            drive.lineToCHeading(rightBProp.getX(), rightBProp.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate ++;
                            }
                        }else if(intermediaterandomizationstate == 3){
                            drive.lineTo(beforeStack.getX(), beforeStack.getY(), beforeStack.getHeading());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate = 0;
                                currentState = State.STACK;
                            }
                        }
                    }
                    break;
                case STACK:

                    drive.lineTo(stackPos.getX(), stackPos.getY(), Math.toRadians(-180));

                    if(drive.isAtTarget()){
                        currentState = State.DEPOSIT;
                    }

                    break;
                case DEPOSIT:
                    if(randomization==0){
                        if(intermediaterandomizationstate == 0){
                            drive.lineToCHeading(dropOff.getX(), dropOff.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 1){
                            drive.lineTo(depositL.getX(), depositL.getY(), depositL.getHeading());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 2){
                            drive.lineTo(afterDepo.getX(), afterDepo.getY(), afterDepo.getHeading());

                            if(drive.isAtTarget()){
                                currentState = State.STACK;
                                intermediaterandomizationstate =0;
                            }
                        }
                    }else if(randomization==1){
                        if(intermediaterandomizationstate == 0){
                            drive.lineToCHeading(dropOff.getX(), dropOff.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 1){
                            drive.lineTo(depositM.getX(), depositM.getY(), depositM.getHeading());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 2){
                            drive.lineTo(afterDepo.getX(), afterDepo.getY(), afterDepo.getHeading());

                            if(drive.isAtTarget()){
                                currentState = State.STACK;
                                intermediaterandomizationstate =0;
                            }
                        }
                    }else{ //right
                        if(intermediaterandomizationstate == 0){
                            drive.lineToCHeading(dropOff.getX(), dropOff.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 1){
                            drive.lineTo(depositR.getX(), depositR.getY(), depositR.getHeading());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 2){
                            drive.lineTo(afterDepo.getX(), afterDepo.getY(), afterDepo.getHeading());

                            if(drive.isAtTarget()){
                                currentState = State.STACK;
                                intermediaterandomizationstate =0;
                            }
                        }
                    }
                    break;


                case PARK:
                    break;
            }
            drive.update();
            telemetry.addData("heading", drive.getR());
            telemetry.addData("target heading", drive.getRTarget());

            telemetry.update();
        }
    }
}
