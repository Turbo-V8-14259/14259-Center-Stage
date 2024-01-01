package org.firstinspires.ftc.teamcode.drive.posePID;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;

@Config
public class DriveTrain {
    private boolean isAtTarget;
    private boolean xOn = true, yOn = true, headingOn = true;
    private double xTol = 1, yTol = 1, headingTol = .5;
    SampleMecanumDrive drive;


    private static PID.Coefficients translationalPidCoefficients = new PID.Coefficients(0.0001, 0.1, 0.01);
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
                DriveTrain.translationalPidCoefficients,
                () -> this.xPred - this.x,
                factor -> this.addPower(factor, this.headingPred));
        this.yPid = new PID(
                DriveTrain.translationalPidCoefficients,
                () -> this.yPred - this.y,
                factor -> this.addPower(factor, this.headingPred));
        this.headingPid = new PID(
                DriveTrain.headingPidCoefficients,
                () -> this.headingPred - this.heading,
                factor -> this.addHeadingPower(factor));
    }

    public void updateLocalizer(){
        drive.updatePoseEstimate();
    }
    public void update() {
        this.updateLocalizer();
        this.xPred = drive.getPoseEstimate().getX();
        this.yPred = drive.getPoseEstimate().getY();
        this.headingPred = drive.getPoseEstimate().getHeading(); //radians

        if(this.xOn){
            this.xPid.update();
        }else{
            this.xPower = 0;
        }
        if(this.yOn){
            this.yPid.update();
        }else{
            this.yPower = 0;
        }
        if(this.headingOn){
            this.headingPid.update();
        }else{
            this.headingPower = 0;
        }
        if(Math.abs(this.x - this.xPred) < xTol){
            this.xPower = 0;
        }
        if(Math.abs(this.y - this.yPred) < yTol){
            this.yPower = 0;
        }
//        this.drive.setWeightedDrivePower(new Pose2d(-xPower, 0, 0));
//        this.drive.setWeightedDrivePower(new Pose2d(0, yPower, 0));
        this.drive.setWeightedDrivePower(new Pose2d(-xPower, yPower, 0));

//        this.drive.setWeightedDrivePower(new Pose2d(-xPower, yPower, headingPower));
//        this.drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        drive.update();
//        if(Math.abs(this.x - this.xPred) < xTol && Math.abs(this.y - this.yPred) < yTol && Math.abs(this.heading - this.headingPred) < headingTol){
//            this.isAtTarget = true;
//        }else{
//            this.isAtTarget = false;
//        }
    }

    public double getX() {
        return this.xPred;
    }
    public double getY() {
        return this.yPred;
    }
    public double getHeading() {
        return this.headingPred;
    }

    public double getTargetX() {
        return this.x;
    }
    public double getTargetY() {
        return this.y;
    }
    public double getTargetHeading() {
        return this.heading;
    }
    public double getDeltaX() {
        return this.x - this.xPred;
    }
    public double getDeltaY() {
        return this.y - this.yPred;
    }
    public double getDeltaHeading() {
        return this.heading - this.headingPred;
    }

    public boolean checkIsAtTarget() {
        return this.isAtTarget;
    }

    public void setxTol(double xTol) {
        this.xTol = xTol;
    }
    public void setyTol(double yTol) {
        this.yTol = yTol;
    }
    public void setheadingTol(double headingTol) {
        this.headingTol = headingTol;
    }

    public void turnXOff(){
        this.xOn = false;
    }
    public void turnXOn(){
        this.xOn = true;
    }
    public void turnYOff(){
        this.yOn = false;
    }
    public void turnYOn(){
        this.yOn = true;
    }
    public void turnHeadingOff(){
        this.headingOn = false;
    }
    public void turnHeadingOn(){
        this.headingOn = true;
    }
    public double getPowerX(){
        return this.xPower;
    }
    public double getPowerY(){
        return this.yPower;
    }
    public double getPowerHeading(){
        return this.headingPower;
    }

    public double getPIDCalculationX(){
        return this.xPred - this.x;
    }

    public double getPIDCalculationY(){
        return this.yPred - this.y;
    }
    public double getPIDCalculationHeading(){
        return this.headingPred - this.heading;
    }


}
