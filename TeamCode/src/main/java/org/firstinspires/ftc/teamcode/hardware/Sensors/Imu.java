package org.firstinspires.ftc.teamcode.hardware.Sensors;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Imu{
    //todo: test
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections = RevHubOrientationOnRobot.UsbFacingDirection.values();

    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    private YawPitchRollAngles angles;
    private IMU imu;

    public Imu(IMU imu){
        this.imu = imu;
    }

    public void init(){
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        this.imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void update(){
       angles = this.imu.getRobotYawPitchRollAngles();
    }


    //always radians
    public double getYawR(){
        return angles.getYaw(AngleUnit.RADIANS);
    }

    public double getPitchR(){
        return angles.getPitch(AngleUnit.RADIANS);
    }

    public double getRollR(){
        return angles.getRoll(AngleUnit.RADIANS);
    }


}
