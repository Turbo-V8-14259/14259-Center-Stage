package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.CameraPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.usefuls.Math.CalculateTangents;



@Autonomous
public class AutoTest extends LinearOpMode {

    enum State{
        IDLE,
        BEFOREPROPID,
        PROPID,
        YELLOWPIXEL,
        INTAKE,
        DEPOSIT,


    }

    SampleMecanumDrive drive;
    CameraPipeline cameraPipeline;
    State currentState = State.IDLE;

    int propID;
    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        cameraPipeline = new CameraPipeline(telemetry, "");

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
                        currentState = State.DEPOSIT;
                        drive.followTrajectoryAsync(depositYellow);
                    }
                    break;
                case DEPOSIT:
                    if(!drive.isBusy()){
                        //next state
                        //robot deposits the yellow pixel here
                    }

            }

            drive.update();
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
