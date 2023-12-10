package org.firstinspires.ftc.teamcode.drive.posePID;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;

public class DriveTrain {
    SampleMecanumDrive drive;


    private static PID.Coefficients xPidCoefficients = new PID.Coefficients(0.08, 0.1, 0.01);
    private static PID.Coefficients yPidCoefficients = new PID.Coefficients(0.08, 0.1, 0.01);
    private static PID.Coefficients headingPidCoefficients = new PID.Coefficients(0.7, 0.55, 0.01);
    private PID xPid, yPid, headingPid;



    private double x = 0.0, y = 0.0, heading = 0.0;
    public double xPred = 0.0, yPred = 0.0, headingPred = 0.0, headingLast = 0.0, headingCurrent = 0.0;
    private int count = 0;
    public void addX(double x) { this.x += x; }
    public void addY(double y) { this.y += y; }
    public void addHeading(double heading) { this.heading += heading; }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }
    public void setHeading(double heading) { this.heading = heading; }
    private void addPower(double power, double r) {
        this.xPower += power * Math.cos(r);
        this.yPower += power * Math.sin(r);
    }
    private void addHeadingPower(double power) {
        this.headingPower += power;
    }
    private double xPower = 0.0, yPower = 0.0, headingPower = 0.0;
    public DriveTrain(HardwareMap hardwareMap) {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.xPid = new PID(
                DriveTrain.xPidCoefficients,
                () -> this.x - this.xPred,
                factor -> this.addPower(factor, -this.headingPred));
        this.yPid = new PID(
                DriveTrain.yPidCoefficients,
                () -> this.y - this.yPred,
                factor -> this.addPower(factor, -this.headingPred + Math.PI / 2.0));
        this.headingPid = new PID(
                DriveTrain.headingPidCoefficients,
                () -> this.heading - this.headingPred,
                factor -> this.addHeadingPower(factor));
    }

    public void updateLocalizer(){
        drive.updatePoseEstimate();
    }
    public void update() {
        this.updateLocalizer();
        this.xPred = drive.getPoseEstimate().getX();
        this.yPred = drive.getPoseEstimate().getY();
        this.headingCurrent = drive.getPoseEstimate().getHeading(); //radians
        if(Math.abs(headingCurrent - headingLast) > M.PI){
            count += Math.signum(headingLast - headingCurrent);
        }
        headingLast = headingCurrent;
        this.headingPred = count * 2*M.PI + headingCurrent;
        this.xPid.update();
        this.yPid.update();
        this.headingPid.update();
        this.drive.setWeightedDrivePower(new Pose2d(xPower, yPower, headingPower));
        drive.update();
    }

}
