package org.firstinspires.ftc.teamcode.opmode.tests.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.posePID2.DT;

@TeleOp(name = "auto test front")
@Config
public class AutoTestFront extends LinearOpMode {
    enum State {
        PROP,
        STACK,
        DEPOSIT
    }
    //props
    Pose2d leftBProp = new Pose2d(-35, -32, Math.toRadians(-180));
    Vector2d leftBPropIntermediate = new Vector2d(-37, -32);
    Vector2d leftFProp = new Vector2d(10, -31);
    Vector2d leftFPropIntermediate = new Vector2d(14,-31);

    Vector2d rightBProp = new Vector2d(-35, -32); //rn
    Vector2d rightBPropIntermediate = new Vector2d(-32, -32); //rn
    Vector2d rightFProp = new Vector2d(31, -32);
    Vector2d rightFPropIntermediate = new Vector2d(38, -32);

    Vector2d middleBProp = new Vector2d(-35, -12);
    Vector2d middleBPropIntermediate = new Vector2d(-35,-14);
    Vector2d middleFProp = new Vector2d(20, -24);


    //stack
    Vector2d beforeStack = new Vector2d(-37, -12);
    Vector2d stackPos = new Vector2d(-54, -12);

    Vector2d FStackI2 = new Vector2d(35, -60);
    Vector2d FStackI1 = new Vector2d(-37, -60);
    Vector2d FStack = new Vector2d(-54, -36);
    //deposit
    Vector2d dropOff = new Vector2d(32, -12);

    Pose2d depositL = new Pose2d(35, -16, Math.toRadians(-200));
    Pose2d depositR = new Pose2d(35, -16, Math.toRadians(-220));
    Pose2d afterDepo = new Pose2d(35, -12, Math.toRadians(-180));

    Pose2d FFirstR = new Pose2d(45, -42, Math.toRadians(-90));
    Vector2d FFirstM = new Vector2d(45, -35);
    Pose2d FFirstL = new Pose2d(45, -28, Math.toRadians(-90));
    Vector2d runToBoardPos = new Vector2d(55, -12);

    AutoTestBack.State currentState;

    int randomization = 0;
    int intermediaterandomizationstate =2;
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap, new Pose2d(-35, -65, Math.toRadians(-90)));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            switch(currentState){
                case PROP:
                    if(randomization==2){
                        if(intermediaterandomizationstate == 0){
                            drive.lineTo(FFirstR.getX(), FFirstR.getY(), FFirstR.getHeading());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 1){
                            drive.lineToCHeading(leftFProp.getX(), leftFProp.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 2){
                            drive.lineToCHeading(leftFPropIntermediate.getX(), leftFPropIntermediate.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 3){
                            drive.lineToCHeading(dropOff.getX(), dropOff.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate = 0;
                                currentState = AutoTestBack.State.STACK;
                            }
                        }
                    }else if(randomization==1){

                    }else{

                    }
                    break;
                case STACK:

                    if(randomization==2){
                        if(intermediaterandomizationstate == 0){
                            drive.lineToCHeading(stackPos.getX(), stackPos.getY());

                            if(drive.isAtTarget()){
                                currentState = AutoTestBack.State.DEPOSIT;
                            }
                        }
                    }else if(randomization==2){

                    }else{

                    }
                    break;
                case DEPOSIT:
                    if(randomization==2){
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
                                currentState = AutoTestBack.State.STACK;
                                intermediaterandomizationstate =0;
                            }
                        }
                    }else if(randomization==1){

                    }else{

                    }
                    break;

            }
            drive.update();
            telemetry.update();
        }
    }
}
