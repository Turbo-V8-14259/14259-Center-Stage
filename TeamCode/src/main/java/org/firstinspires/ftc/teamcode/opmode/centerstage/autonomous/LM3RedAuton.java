package org.firstinspires.ftc.teamcode.opmode.centerstage.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.posePID2.DT;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Claw;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.hardware.Sensors.Blinkdin;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

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
    Pose2d startBPose = new Pose2d(20, -65, Math.toRadians(-90)); //default position
    int randomization;
    int intermediate0 = 0;
    int intermediate1 = 0;
    @Override
    public void runOpMode() throws InterruptedException {
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
        randomization = 0;
        waitForStart();
        while(opModeIsActive()){

            if(randomization == 0){ //LEFT


                if(intermediate0 == 0){
                    drive.lineTo(60, -45, Math.toRadians(-180));
                    if(drive.isAtTarget()) intermediate0++;
                }else if(intermediate0==1){
                    intake.setState(Intake.IntakeState.INTAKE_TELE);
                    drive.lineTo(60, -32, Math.toRadians(-180));
                    if(drive.isAtTarget()) intermediate0++;
                }//DRIVES TO THE RANDOMIZATION BOARD LOCATION

                else if(intermediate0==2){ //ARM SCORING
                    arm.setState(DepoArm.DepoArmState.ABSOLUTE_INTERMEDIATE);
                    timerShit(1000);

                }else if(intermediate0==3){
                    turret.setState(LM1Turret.TurretState.SCORE);
                    timerShit(1000);
                }else if(intermediate0 ==4){
                    slides.setState(DepoSlides.DepositState.AUTO_PRELOAD_SCORE);
                    timerShit(1000);
                }else if(intermediate0 ==5){
                    claw.setState(Claw.ClawState.UNLATCHED);
                    timerShit(500);

                }else if(intermediate0 ==6){
                    slides.setState(DepoSlides.DepositState.DOWN);
                    timerShit(1000);
                }else if(intermediate0 ==7){
                    turret.setState(LM1Turret.TurretState.INITIALIZE);
                    timerShit(1000);
                }else if(intermediate0 ==8){
                    arm.setState(DepoArm.DepoArmState.INITIALIZE);
                    timerShit(1000);
                }else if(intermediate0 == 9){ //GO TO POSITION SPIKE
                    drive.lineTo(27, -39, Math.toRadians(-180));
                    if(drive.isAtTarget()) intermediate0++;
                } else if(intermediate0==10){
                    intake.intakeMotor.setPower(0.5);
                    timerShit(1000);
                }else if(intermediate0 == 11){
                    drive.lineTo(55,-60,Math.toRadians(-180)); ///PARK
                }



            }
            else if(randomization == 1){ //MIDDLE




                if(intermediate1 == 0){
                    drive.lineTo(60, -45, Math.toRadians(-180));
                    if(drive.isAtTarget()) intermediate1++;
                }else if(intermediate1==1){
                    intake.setState(Intake.IntakeState.INTAKE_TELE);
                    drive.lineTo(60, -37.5, Math.toRadians(-180));
                }//DRIVES TO THE RANDOMIZATION BOARD LOCATION




            }
            else if(randomization == 2){ //RIGHT
                intake.setState(Intake.IntakeState.INTAKE_TELE);




                drive.lineTo(60, -45, Math.toRadians(-180));
                //DRIVES TO THE RANDOMIZATION BOARD LOCATION





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
}
