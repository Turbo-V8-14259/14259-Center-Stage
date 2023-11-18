package org.firstinspires.ftc.teamcode.hardware;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoTurret;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
//import org.firstinspires.ftc.teamcode.hardware.Sensors.Blinkdin;
import org.firstinspires.ftc.teamcode.hardware.Sensors.Imu;
import org.firstinspires.ftc.teamcode.hardware.Sensors.axonEncoder;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;

public class Robot {
    public SampleMecanumDrive drive;
    public DepoSlides depoSlides;
    public Pitch pitch;
    public DepoTurret depoTurret;
    public Intake intake;
    public Imu imu;
    public axonEncoder turretEncoder;
//    public Blinkdin LED;
    public stickyGamepad gamepadOne;
    public stickyGamepad gamepadTwo;
    public Robot(SampleMecanumDrive drive, DepoSlides depoSlides, Pitch pitch, DepoTurret depoTurret, Intake intake, Imu imu, axonEncoder turretEncoder, stickyGamepad gamepadOne, stickyGamepad gamepadTwo){ {
        this.drive = drive;
        this.depoSlides = depoSlides;
        this.pitch = pitch;
        this.depoTurret = depoTurret;
        this.intake = intake;
        this.turretEncoder = turretEncoder;
//        this.LED = LED;

        this.imu = imu;
        this.imu.init();

        this.gamepadOne = gamepadOne;
        this.gamepadTwo = gamepadTwo;
        }
    }


}
