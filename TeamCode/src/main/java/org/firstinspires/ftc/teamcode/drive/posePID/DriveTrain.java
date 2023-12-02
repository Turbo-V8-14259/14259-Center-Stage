package org.firstinspires.ftc.teamcode.drive.posePID;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;

public class DriveTrain {
    SampleMecanumDrive drive;


    private static PID.Coefficients xPidCoefficients = new PID.Coefficients(0.08, 0.1, 0.01);
    private static PID.Coefficients yPidCoefficients = new PID.Coefficients(0.08, 0.1, 0.01);
    private static PID.Coefficients headingPidCoefficients = new PID.Coefficients(0.7, 0.55, 0.01);
    private PID xPid, yPid, headingPid;



    private double x = 0.0, y = 0.0, heading = 0.0;
    private double xPred = 0.0, yPred = 0.0, headingPred = 0.0, headingLast = 0.0, headingCurrent = 0.0;
    private int count = 0;
    public void addX(double x) { this.x += x; }
    public void addY(double y) { this.y += y; }
    public void addHeading(double heading) { this.heading += heading; }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }
    public void setHeading(double heading) { this.heading = heading; }

    private double xPower = 0.0, yPower = 0.0, headingPower = 0.0;
    public DriveTrain(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    public void updateLocalizer(){
        drive.updatePoseEstimate();
    }
    public void update() {
        this.updateLocalizer();
        this.xPred = drive.getPoseEstimate().getX();
        this.yPred = drive.getPoseEstimate().getY();
        this.headingCurrent = drive.getPoseEstimate().getHeading(); //degree? radian?
        if(Math.abs(headingCurrent - headingLast) > 180){
            count += Math.signum(headingLast - headingCurrent);
        }
        headingLast = headingCurrent;
        this.headingPred = count * 360 + headingCurrent;
        this.xPid.update();
        this.yPid.update();
        this.headingPid.update();
        this.drive.setWeightedDrivePower(new Pose2d(xPower, yPower, headingPower));
        drive.update();
    }
}
