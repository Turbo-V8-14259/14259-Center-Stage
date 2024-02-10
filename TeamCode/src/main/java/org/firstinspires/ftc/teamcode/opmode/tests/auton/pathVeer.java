package org.firstinspires.ftc.teamcode.opmode.tests.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.posePID2.DT;

@TeleOp(name = "pathVeer")
@Config
@Disabled
public class pathVeer extends LinearOpMode {

    enum State {
        PROP,
        STACK,
        DEPOSIT,
        END
    }
    int x=0;
    int y=-12;
    //start locs
    Pose2d startBPose = new Pose2d(-30+x, -65+y, Math.toRadians(-90)); //default position
    Pose2d startFPose = new Pose2d(11+x, -65+y, Math.toRadians(-90));
    //props
    Pose2d leftBProp = new Pose2d(-40+x, -40+y, Math.toRadians(-180));
    Vector2d leftBPropIntermediate = new Vector2d(-37+x, -32+y);
    Vector2d leftFProp = new Vector2d(10+x, -31+y);
    Vector2d leftFPropIntermediate = new Vector2d(14+x,-31+y);

    Vector2d rightBProp = new Vector2d(-35+x, -32+y); //rn
    Vector2d rightBPropIntermediate = new Vector2d(-32+x, -32+y); //rn
    Vector2d rightFProp = new Vector2d(31+x, -32+y);
    Vector2d rightFPropIntermediate = new Vector2d(38+x, -32+y);

    Vector2d middleBProp = new Vector2d(-35+x, -12+y);
    Vector2d middleBPropIntermediate = new Vector2d(-35+x,-14+y);
    Vector2d middleFProp = new Vector2d(20+x, -24+y);


    //stack
    Vector2d beforeStack = new Vector2d(-37+x, -12+y);
    Vector2d stackPos = new Vector2d(-54+x, -12+y);

    Vector2d FStackI2 = new Vector2d(35+x, -60+y);
    Vector2d FStackI1 = new Vector2d(-37+x, -60+y);
    Vector2d FStack = new Vector2d(-54+x, -36+y);
    //deposit
    Vector2d dropOff = new Vector2d(32+x, -12+y);

    Pose2d depositL = new Pose2d(35+x, -16+y, Math.toRadians(-200));
    Pose2d afterDepo = new Pose2d(35+x, -12+y, Math.toRadians(-180));

    Pose2d FFirstR = new Pose2d(45+x, -42+y, Math.toRadians(-90));
    Vector2d FFirstM = new Vector2d(45+x, -35+y);
    Pose2d FFirstL = new Pose2d(45+x, -28+y, Math.toRadians(-90));
    Vector2d runToBoardPos = new Vector2d(55+x, -12+y);

    State currentState;

    int randomization = 0;
    int intermediaterandomizationstate =0;
    int a = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap, new Pose2d(startBPose.getX(), startBPose.getY(), startBPose.getHeading()));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        currentState  = State.PROP;
        waitForStart();
        while(opModeIsActive()){
            if(a == 0){
                drive.lineTo(leftBProp.getX(), leftBProp.getY(), leftBProp.getHeading());
                if(drive.isAtTarget()) a++;
            }
//            else if(a==1){
//                drive.lineTo(leftBPropIntermediate.getX(), leftBPropIntermediate.getY(), leftBProp.getHeading());
//                if(drive.isAtTarget()) a++;
//            }else if(a==2){
//                drive.lineTo(leftBProp.getX(), leftBProp.getY(), leftBProp.getHeading());
//                if(drive.isAtTarget()) a++;
//            }else if(a==3){
//                drive.lineTo(beforeStack.getX(), beforeStack.getY(), leftBProp.getHeading());
//                if(drive.isAtTarget()) a++;
//            }
//            switch(currentState){
//                case PROP:
//                    if(randomization==0){
//                        if(intermediaterandomizationstate == 0){
//                            drive.lineTo(leftBProp.getX(), leftBProp.getY(), leftBProp.getHeading());
//
//                            if(drive.isAtTarget()){
//                                intermediaterandomizationstate++;
//                            }
//                        }else if(intermediaterandomizationstate == 1){
//                            drive.lineToCHeading(leftBPropIntermediate.getX(), leftBPropIntermediate.getY());
//
//                            if(drive.isAtTarget()){
//                                intermediaterandomizationstate++;
//                            }
//                        }else if(intermediaterandomizationstate == 2){
//                            drive.lineToCHeading(leftBProp.getX(), leftBProp.getY());
//
//                            if(drive.isAtTarget()){
//                                intermediaterandomizationstate++;
//                            }
//                        }else if(intermediaterandomizationstate == 3){
//                            drive.lineToCHeading(beforeStack.getX(), beforeStack.getY());
//
//                            if(drive.isAtTarget()){
//                                intermediaterandomizationstate = 0;
//                                currentState = State.STACK;
//                            }
//                        }
//                    }else if(randomization==1){
//
//                    }else{
//
//                    }
//                    break;
//                case STACK:
//
//                    if(randomization==0){
//                        if(intermediaterandomizationstate == 0){
//                            drive.lineTo(stackPos.getX(), stackPos.getY(), Math.toRadians(-180));
//
//                            if(drive.isAtTarget()){
//                                currentState = State.DEPOSIT;
//                            }
//                        }
//                    }else if(randomization==1){
//
//                    }else{
//
//                    }
//                    break;
//                case DEPOSIT:
//                    if(randomization==0){
//                        if(intermediaterandomizationstate == 0){
//                            drive.lineToCHeading(dropOff.getX(), dropOff.getY());
//
//                            if(drive.isAtTarget()){
//                                intermediaterandomizationstate++;
//                            }
//                        }else if(intermediaterandomizationstate == 1){
//                            drive.lineTo(depositL.getX(), depositL.getY(), depositL.getHeading());
//
//                            if(drive.isAtTarget()){
//                                intermediaterandomizationstate++;
//                            }
//                        }else if(intermediaterandomizationstate == 2){
//                            drive.lineTo(afterDepo.getX(), afterDepo.getY(), afterDepo.getHeading());
//
//                            if(drive.isAtTarget()){
//                                currentState = State.END;
//                                intermediaterandomizationstate =0;
//                            }
//                        }
//                    }else if(randomization==1){
//
//                    }else{
//
//                    }
//                    break;
//
//
//                case END:
//                    break;
//            }
            drive.update();
            telemetry.addData("heading", drive.getR());
            telemetry.addData("target heading", drive.getRTarget());

            telemetry.update();
        }
    }
}
