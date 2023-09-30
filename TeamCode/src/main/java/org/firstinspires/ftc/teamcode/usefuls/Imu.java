package org.firstinspires.ftc.teamcode.usefuls;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Imu{
    //todo: test

    public HardwareMap hardwareMap;

    public IMU imu;

    YawPitchRollAngles orientation;

    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
    }

    public void update(){
       orientation = imu.getRobotYawPitchRollAngles();
    }


    //always radians

    public double getYawR(){
        update();
        return orientation.getYaw(AngleUnit.RADIANS);

    }

    public double getPitchR(){
        update();
        return orientation.getPitch(AngleUnit.RADIANS);
    }

    public double getRollR(){
        update();
        return orientation.getRoll(AngleUnit.RADIANS);
    }


}
