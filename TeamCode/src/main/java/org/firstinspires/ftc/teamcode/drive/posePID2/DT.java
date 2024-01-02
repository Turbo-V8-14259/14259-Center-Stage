package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
public class DT{
    private SampleMecanumDrive drive;
    private DcMotorEx leftFront;
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;

    private double xTarget;
    private double yTarget;
    private double rTarget;

    private double xRn, yRn, rRn;

    private BasicPID xController, yController, rController;
    private PIDCoefficients xyCoeff, rCoeff;
    public static double xyP = 0.15, xyI = 0, xyD = .2;
    public static double rP = 0.5, rI = 0, rD = 0;

    private double xOut, yOut, rOut;
    private double twistedR, count, lastAngle;
    private double xPower, yPower;

    public DT(HardwareMap hardwareMap){
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        this.rightRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftRear.setDirection(DcMotor.Direction.REVERSE);
        this.xyCoeff = new PIDCoefficients(xyP, xyI, xyD);
        this.rCoeff = new PIDCoefficients(rP, rI, rD);
        this.xController = new BasicPID(xyCoeff);
        this.yController = new BasicPID(xyCoeff);
        this.rController = new BasicPID(rCoeff);
        this.xTarget = 0;
        this.yTarget = 0;
        this.rTarget = 0;
    }
    public DT(HardwareMap hardwareMap, Pose2d startPose){
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.drive.setPoseEstimate(startPose);
        this.leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");

        this.rightRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftRear.setDirection(DcMotor.Direction.REVERSE);
        this.xyCoeff = new PIDCoefficients(xyP, xyI, xyD);
        this.rCoeff = new PIDCoefficients(rP, rI, rD);
        this.xController = new BasicPID(xyCoeff);
        this.yController = new BasicPID(xyCoeff);
        this.rController = new BasicPID(rCoeff);
        this.xTarget = startPose.getX();
        this.yTarget = startPose.getY();
        this.rTarget = startPose.getHeading();
    }
    public void setPowers(double y, double x, double r){
        double normalize = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r),1);
        leftFront.setPower(((y+x+r)/normalize));
        leftRear.setPower(((y-x+r)/normalize));
        rightRear.setPower(((y+x-r)/normalize));
        rightFront.setPower(((y-x-r)/normalize));
    }
    public void update(){
        drive.updatePoseEstimate();
        xRn = drive.getPoseEstimate().getX();
        yRn = drive.getPoseEstimate().getY();
        rRn = drive.getPoseEstimate().getHeading();
        xOut = xController.calculate(xTarget, xRn);
        yOut = yController.calculate(yTarget, yRn);
        if(Math.abs(rRn - lastAngle) > Math.PI){
            count += Math.signum(lastAngle - rRn);
        }
        lastAngle = rRn;
        twistedR = count * (2*Math.PI) + rRn;
        rOut = rController.calculate(rTarget, twistedR);
        xPower = xOut * Math.cos(rRn) - (-yOut) * Math.sin(rRn);
        yPower = xOut * Math.sin(rRn) + (-yOut) * Math.cos(rRn);
        setPowers(xPower, yPower,-rOut);
    }

    public void setXTarget(double x){
        this.xTarget = x;
    }
    public void setYTarget(double y){
        this.yTarget = y;
    }
    public void setRTarget(double r){
        this.rTarget = r;
    }
    public double getPowerX(){
        return xPower;
    }
    public double getPowerY(){
        return yPower;
    }
    public double getPowerR(){
        return -rOut;
    }
    public double getRawY(){
        return yOut;
    }
    public double getRawX(){
        return xOut;
    }
    public double getX(){
        return xRn;
    }
    public double getY(){
        return yRn;
    }
    public double getR(){
        return rRn;
    }
    public double getYTarget(){
        return yTarget;
    }
    public double getTwistedR(){
        return twistedR;
    }
    public void lineTo(double x, double y, double r){
        setXTarget(x);
        setYTarget(y);
        setRTarget(r);
    }
    public void lineToCHeading(double x, double y, double r){
        setXTarget(x);
        setYTarget(y);
    }
}
