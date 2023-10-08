package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Imu{
    //todo: test
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();

    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    private HardwareMap hardwareMap;

    private IMU imu;

    YawPitchRollAngles angles;

    public void init() {
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void update(){
       angles = imu.getRobotYawPitchRollAngles();
    }


    //always radians

    public double getYawR(){
        update();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    public double getPitchR(){
        update();
        return angles.getPitch(AngleUnit.RADIANS);
    }

    public double getRollR(){
        update();
        return angles.getRoll(AngleUnit.RADIANS);
    }


}
