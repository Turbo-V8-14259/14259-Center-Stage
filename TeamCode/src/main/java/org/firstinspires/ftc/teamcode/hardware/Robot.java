package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Claw;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoArm;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoTurret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.LM1Turret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
//import org.firstinspires.ftc.teamcode.hardware.Sensors.Blinkdin;
import org.firstinspires.ftc.teamcode.hardware.Sensors.Blinkdin;
import org.firstinspires.ftc.teamcode.hardware.Sensors.Imu;
import org.firstinspires.ftc.teamcode.hardware.Sensors.axonEncoder;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;

public class Robot {
    public enum RobotFSM{
        IDLE,
        INTAKE,
        SCORINGFSM,
        HANG
    }
    public RobotFSM robotState = RobotFSM.IDLE;
    Pitch pitch;
    DepoSlides slides;
    Blinkdin led;
    SampleMecanumDrive drive;
    Intake intake;
    Claw claw;
    LM1Turret turret;
    DepoArm arm;
    stickyGamepad gamepadOne;
    stickyGamepad gamepadTwo;
    ElapsedTime timer;
    ElapsedTime timer2;
    public Robot(SampleMecanumDrive drive, DepoSlides depoSlides, Pitch pitch, LM1Turret depoTurret, DepoArm arm, Claw claw, Intake intake, stickyGamepad gamepadOne, stickyGamepad gamepadTwo, Blinkdin led, ElapsedTime timer, ElapsedTime timer2){
        this.drive = drive;
        this.slides = depoSlides;
        this.pitch = pitch;
        this.turret = depoTurret;
        this.intake = intake;
        this.claw = claw;
        this.arm = arm;
        this.gamepadOne = gamepadOne;
        this.gamepadTwo = gamepadTwo;
        this.timer = timer;
        this.timer2 = timer2;
        this.led = led;
    }

    public Robot.RobotFSM getState(){
        return  robotState;
    }

    public void setState(RobotFSM state){
        this.robotState = state;
        switch (robotState){
            case IDLE:
                break;
            case INTAKE:
                break;
            case SCORINGFSM:
                break;
        }
    }



}
