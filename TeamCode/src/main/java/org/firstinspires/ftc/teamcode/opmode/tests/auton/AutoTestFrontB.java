package org.firstinspires.ftc.teamcode.opmode.tests.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.posePID2.DT;

@TeleOp(name = "auto test front blue")
@Config
public class AutoTestFrontB extends LinearOpMode {
    enum State {
        PROP,
        STACK,
        DEPOSIT,
        END
    }
    //start locs
    int x =0;
    int y = 0;
    Pose2d startFPose = new Pose2d(11+x, 65+y, Math.toRadians(90));
    //props
    Vector2d leftFProp = new Vector2d(10+x, 31+y);
    Vector2d leftFPropIntermediate = new Vector2d(14+x,31+y);

    Vector2d rightFProp = new Vector2d(31+x, 32+y);
    Vector2d rightFPropIntermediate = new Vector2d(38+x, 32+y);

    Vector2d middleFProp = new Vector2d(20+x, 24+y);


    //stack
    Vector2d beforeStack = new Vector2d(-37, 12);
    Vector2d stackPos = new Vector2d(-54, 12);

    Pose2d FStackI2 = new Pose2d(35+x, 60+y);
    Pose2d FStackI1 = new Pose2d(-37+x, 60+y);
    Pose2d FStack = new Pose2d(-54+x, 36+y);
    //deposit
    Vector2d dropOff = new Vector2d(32, 12);
    Pose2d depositL = new Pose2d(35+x, 16+y, Math.toRadians(200));
    Pose2d depositM = new Pose2d(35+x, 16+y, Math.toRadians(210));
    Pose2d depositR = new Pose2d(35+x, 16+y, Math.toRadians(220));
    Pose2d afterDepo = new Pose2d(35+x, 12+y, Math.toRadians(180));
    Pose2d FFirstR = new Pose2d(45+x, 42+y, Math.toRadians(180));
    Pose2d FFirstM = new Pose2d(45+x, 35+y, Math.toRadians(180));
    Pose2d FFirstL = new Pose2d(45+x, 28+y, Math.toRadians(180));
    Vector2d runToBoardPos = new Vector2d(55, 12);

    State currentState;

    int randomization = 0;
    int intermediaterandomizationstate =2;
    @Override
    public void runOpMode() throws InterruptedException {
        DT drive = new DT(hardwareMap, new Pose2d(startFPose.getX(), startFPose.getY(), startFPose.getHeading()));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        currentState = State.PROP;
        waitForStart();
        while(opModeIsActive()){
            switch(currentState){
                case PROP:
                    if(randomization==0){
                        if(intermediaterandomizationstate == 0){
                            drive.lineTo(FFirstL.getX(), FFirstL.getY(), FFirstL.getHeading());

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
                        }
                        else if(intermediaterandomizationstate == 3){
                            drive.lineToCHeading(dropOff.getX(), dropOff.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate = 0;
                                currentState = State.STACK;
                            }
                        }
                    }else if(randomization==1){
                        if(intermediaterandomizationstate == 0){
                            drive.lineTo(FFirstM.getX(), FFirstM.getY(), FFirstM.getHeading());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 1){
                            drive.lineToCHeading(middleFProp.getX(), middleFProp.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 2){
                            drive.lineToCHeading(dropOff.getX(), dropOff.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate = 0;
                                currentState = State.STACK;
                            }
                        }
                    }else{
                        if(intermediaterandomizationstate == 0){
                            drive.lineTo(FFirstR.getX(), FFirstR.getY(), FFirstR.getHeading());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 1){
                            drive.lineToCHeading(rightFProp.getX(), rightFProp.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 2){
                            drive.lineToCHeading(rightFPropIntermediate.getX(), rightFPropIntermediate.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate++;
                            }
                        }else if(intermediaterandomizationstate == 3){
                            drive.lineToCHeading(dropOff.getX(), dropOff.getY());

                            if(drive.isAtTarget()){
                                intermediaterandomizationstate = 0;
                                currentState = State.STACK;
                            }
                        }
                    }
                    break;
                case STACK:

                    drive.lineTo(stackPos.getX(), stackPos.getY(),FFirstR.getHeading());

                    if(drive.isAtTarget()) {
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
                                currentState = State.DEPOSIT;
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
                                currentState = State.DEPOSIT;
                                intermediaterandomizationstate =0;
                            }
                        }
                    }else{
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
                                currentState = State.DEPOSIT;
                                intermediaterandomizationstate =0;
                            }
                        }
                    }
                    break;

            }
            drive.update();
            telemetry.update();
        }
    }
}
