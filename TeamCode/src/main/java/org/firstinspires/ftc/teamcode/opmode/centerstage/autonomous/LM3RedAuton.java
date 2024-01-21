package org.firstinspires.ftc.teamcode.opmode.centerstage.autonomous;

import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.color;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.leftPer;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.midPer;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.rightPer;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.setColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Claw;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

import org.firstinspires.ftc.teamcode.vision.CameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class LM3RedAuton extends LinearOpMode {
    double TimeStamp = 0;
    double TimeStamp2 = 0;
    double TimeStamp3 = 0;
    boolean timeToggle = true;
    boolean timeToggle2 = true;
    boolean timeToggle3 = true;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();
    Pitch pitch;
    DepoSlides slides;
    Intake intake;
    Claw claw;
    LM1Turret turret;
    DepoArm arm;
    DT drive;
    int x =0;
    Pose2d startBPose = new Pose2d(19+x, -65, Math.toRadians(-90)); //default position
    int randomization;
    int intermediate0 = 0;
    int intermediate1 = 0;

    //vision
    OpenCvWebcam webcam;
    public String ObjectDirection;

    private VisionPortal visionPortal;
    public double thresh = 10;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        claw = new Claw(new ServoMotorBetter(hardwareMap.get(Servo.class, "claw")));
        claw.setState(Claw.ClawState.LATCHED);
        claw.update();
        pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "Pitch")));
        pitch.manualMode = false;
        drive = new DT(hardwareMap, new Pose2d(startBPose.getX(), startBPose.getY(), startBPose.getHeading()));
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "Intake"), new ServoMotorBetter(hardwareMap.get(Servo.class, "intakeArm")));
        intake.setState(Intake.IntakeState.INITIALIZE);
        intake.update();
        turret = new LM1Turret(new ServoMotorBetter(hardwareMap.get(Servo.class, "turret")));
        arm = new DepoArm(new ServoMotorBetter(hardwareMap.get(Servo.class, "arm")), new ServoMotorBetter(hardwareMap.get(Servo.class, "fake")));
        slides = new DepoSlides(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "leftSlides")), new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "rightSlides")));
        slides.passive = false;
        slides.pidRunning = true;
        slides.manualMode = false;

        //vision
        telemetry.addLine("Loading Pipeline...");
        setColor("RED");
        telemetry.update();
        initPipeline();

        //vision math
        while(opModeInInit()) {
            if (leftPer > thresh || rightPer > thresh || midPer > thresh) {
                if (leftPer > rightPer && leftPer > midPer) { //mid
                    if (color.equals("RED")) {
                        ObjectDirection = "LEFT";
                    } else if (color.equals("BLUE")) {
                        ObjectDirection = "MIDDLE";
                    }
                } else if (rightPer > leftPer && rightPer > midPer) { //right
                    if (color.equals("RED")) {
                        ObjectDirection = "MIDDLE";
                    } else if (color.equals("BLUE")) {
                        ObjectDirection = "RIGHT";
                    }
                }
            } else {
                if (color.equals("RED")) {
                    ObjectDirection = "RIGHT";
                } else if (color.equals("BLUE")) {
                    ObjectDirection = "LEFT";
                }
            }

            switch (ObjectDirection) {
                case "LEFT":
                    randomization = 0;
                    break;
                case "RIGHT":
                    randomization = 2;
                    break;
                case "MIDDLE":
                    randomization = 1;
                    break;
            }
            telemetry.addData("Location:", ObjectDirection);
            telemetry.addData("Color:", color);
            telemetry.update();

        }

        waitForStart();
        while(opModeIsActive()){

            if(randomization == 0){ //LEFT
                if(intermediate0 == 0){
                    drive.lineTo(60+x, -45, Math.toRadians(-180));
                    if(drive.isAtTarget()) intermediate0++;
                }else if(intermediate0==1){
                    intake.setState(Intake.IntakeState.INITIALIZE);
                    arm.setState(DepoArm.DepoArmState.AUTO_PRELOAD);
                    drive.lineTo(60+x, -31.5, Math.toRadians(-180));
                    timerShit(2000);
                }//DRIVES TO THE RANDOMIZATION BOARD LOCATION

                else if(intermediate0==2){ //ARM SCORING
                    turret.setState(LM1Turret.TurretState.SCORE);
                    slides.setState(DepoSlides.DepositState.AUTO_PRELOAD_SCORE);
//                    arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                    timerShit(1000);

                }else if(intermediate0==3){
//                    turret.setState(LM1Turret.TurretState.SCORE);
//                    slides.setState(DepoSlides.DepositState.AUTO_PRELOAD_SCORE);
                    timerShit(1);
                }else if(intermediate0 ==4){
//                    slides.setState(DepoSlides.DepositState.AUTO_PRELOAD_SCORE); //merged to previous state
                    claw.setState(Claw.ClawState.UNLATCHED);

                    timerShit(500);
                }else if(intermediate0 ==5){
                    drive.lineTo(56+x, -31.5, Math.toRadians(-180));

                    timerShit(1000);
                }else if(intermediate0 ==6){
                    slides.setState(DepoSlides.DepositState.DOWN);
                    intake.setState(Intake.IntakeState.INITIALIZE);
                    timerShit(1000);
                }else if(intermediate0 ==7){
                    turret.setState(LM1Turret.TurretState.INITIALIZE);
                    timerShit(1000);
                }else if(intermediate0 ==8){
                    arm.setState(DepoArm.DepoArmState.INITIALIZE);
                    timerShit(1000);
                }else if(intermediate0 == 9){ //GO TO POSITION SPIKE
                    drive.lineTo(30+x, -37, Math.toRadians(-180));
                    if(drive.isAtTarget()) intermediate0++;
                } else if(intermediate0==10){
                    intake.intakeMotor.setPower(0.5);
                    timerShit(1000);
                }else if(intermediate0 == 11){
                    drive.lineTo(54+x,-60,Math.toRadians(-180)); ///PARK
                }






            }
            else if(randomization == 1){ //MIDDLE
                if(intermediate0 == 0){
                    drive.lineTo(60+x, -45, Math.toRadians(-180));
                    if(drive.isAtTarget()) intermediate0++;
                }else if(intermediate0==1){
                    arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                    intake.setState(Intake.IntakeState.INITIALIZE);
                    drive.lineTo(60+x, -37, Math.toRadians(-180));
                    timerShit(2000);
                }//DRIVES TO THE RANDOMIZATION BOARD LOCATION

                else if(intermediate0==2){ //ARM SCORING
                    turret.setState(LM1Turret.TurretState.SCORE);
                    slides.setState(DepoSlides.DepositState.AUTO_PRELOAD_SCORE);
//                    arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                    timerShit(1000);

                }else if(intermediate0==3){
                    claw.setState(Claw.ClawState.UNLATCHED);
//                    turret.setState(LM1Turret.TurretState.SCORE);
//                    slides.setState(DepoSlides.DepositState.AUTO_PRELOAD_SCORE);
                    timerShit(500);
                }else if(intermediate0 ==4){
//                    slides.setState(DepoSlides.DepositState.AUTO_PRELOAD_SCORE);
                    drive.lineTo(56+x, -37, Math.toRadians(-180));

                    timerShit(1000);
                }else if(intermediate0 ==5){
                    timerShit(500);
                }else if(intermediate0 ==6){
                    slides.setState(DepoSlides.DepositState.DOWN);
                    turret.setState(LM1Turret.TurretState.INITIALIZE);
                    timerShit(1000);
                }else if(intermediate0 ==7){
//                    turret.setState(LM1Turret.TurretState.INITIALIZE);

                    timerShit(1);
                }else if(intermediate0 ==8){
                    arm.setState(DepoArm.DepoArmState.INITIALIZE);
                    timerShit(1000);
                }else if(intermediate0 == 9){ //GO TO POSITION SPIKE
                    drive.lineTo(40+x, -27, Math.toRadians(-180));
                    if(drive.isAtTarget()) intermediate0++;
                } else if(intermediate0==10){
                    intake.intakeMotor.setPower(0.65);
                    timerShit(1000);
                }else if(intermediate0 == 11){
                    drive.lineTo(54+x,-60,Math.toRadians(-180)); ///PARK
                }



            }
            else if(randomization == 2){ //RIGHT


                if(intermediate0 == 0){
                    drive.lineTo(60+x, -45, Math.toRadians(-180));
                    if(drive.isAtTarget()) intermediate0++;
                }else if(intermediate0==1){
                    arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                    drive.lineTo(60+x, -43.5, Math.toRadians(-180));
//                    if(drive.isAtTarget()) intermediate0++;
                    timerShit(2000);
                }//DRIVES TO THE RANDOMIZATION BOARD LOCATION

                else if(intermediate0==2){ //ARM SCORING
//                    arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                    timerShit(1000);

                }else if(intermediate0==3){
                    turret.setState(LM1Turret.TurretState.SCORE);
                    slides.setState(DepoSlides.DepositState.AUTO_PRELOAD_SCORE);
                    timerShit(1000);
                }else if(intermediate0 ==4){
                    claw.setState(Claw.ClawState.UNLATCHED);

//                    slides.setState(DepoSlides.DepositState.AUTO_PRELOAD_SCORE);
                    timerShit(500);
                }else if(intermediate0 ==5){
                    claw.setState(Claw.ClawState.UNLATCHED);
                    drive.lineTo(56+x, -43.5, Math.toRadians(-180));

                    timerShit(2000);
                }else if(intermediate0 ==6){
                    slides.setState(DepoSlides.DepositState.DOWN);
                    turret.setState(LM1Turret.TurretState.INITIALIZE);
                    intake.setState(Intake.IntakeState.INITIALIZE);
                    timerShit(1000);
                }else if(intermediate0 ==7){
                    timerShit(1);
                }else if(intermediate0 ==8){
                    arm.setState(DepoArm.DepoArmState.INITIALIZE);
                    timerShit(1000);
                }else if(intermediate0 == 9){ //GO TO POSITION SPIKE
                    drive.lineTo(44+x, -37, Math.toRadians(-180));//fucked up
                    if(drive.isAtTarget()) intermediate0++;
                } else if(intermediate0==10){
                    intake.intakeMotor.setPower(0.5);
                    timerShit(1000);
                }else if(intermediate0 == 11){
                    drive.lineTo(54+x,-60,Math.toRadians(-180)); ///PARK
                }





            }
            drive.update();
            turret.update();
            arm.update();
            drive.update();
            intake.update();
            pitch.update();
            slides.update();
            claw.update();
        }
    }
    public void preloadScore(){

    }
    public void timerShit(double time){
        if(timeToggle){//timeToggle starts at true by default
            TimeStamp = timer.milliseconds();
            timeToggle = false;
        }
        if(timer.milliseconds() > TimeStamp + time){
            intermediate0++;
            timeToggle = true;
        }
    }
    private void initPipeline(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        CameraPipeline s = new CameraPipeline(telemetry);
        webcam.setPipeline(s);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        sleep(1000);

    }

    public static class LM3BlueAuton {
    }
}