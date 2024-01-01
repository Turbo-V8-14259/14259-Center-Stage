package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;
@Config
public class DT{
    private SampleMecanumDrive drive;
    private DcMotorEx leftFront;
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;

    private double xTarget = 0;
    private double yTarget = 0;
    private double rTarget = 0;

    private double xRn, yRn, rRn;
    private PIDCoefficients xyCoeff, rCoeff;

    public static double xyP = 0.15, xyI = 0, xyD = .2;
    public static double rP = 0.5, rI = 0, rD = 0;

    private BasicPID xController, yController, rController;

    private double xOut, yOut, rOut;
    private double twistedR, count, lastAngle;

    private double xPower, yPower;


    public DT(HardwareMap hardwareMap){
        this.drive = new SampleMecanumDrive(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");

        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        xyCoeff = new PIDCoefficients(xyP, xyI, xyD);
        rCoeff = new PIDCoefficients(rP, rI, rD);
        xController = new BasicPID(xyCoeff);
        yController = new BasicPID(xyCoeff);
        rController = new BasicPID(rCoeff);

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
        xPower = xOut * Math.cos(rRn) - (-1*yOut) * Math.sin(rRn);
        yPower = xOut * Math.sin(rRn) + (-1*yOut) * Math.cos(rRn);
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
        return (xOut * Math.cos(rRn));
    }
    public double getPowerY(){
        return -1 * yOut * (Math.sin(rRn + Math.PI/2));
    }

    public double getRawY(){
        return yOut;
    }
    public double getRawX(){
        return xOut;
    }
    public double getPowerR(){
        return rOut;
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
}
