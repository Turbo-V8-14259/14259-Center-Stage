package org.firstinspires.ftc.teamcode.drive.posePID2;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Math.T;

@Config
public class DT{
    private SampleMecanumDrive drive;
    private VoltageSensor vs;
    private double compensator;
    private DcMotorEx leftFront, rightRear, rightFront, leftRear;

    private double xTarget, yTarget, rTarget;
    private double xRn, yRn, rRn;
    private double deltaX, deltaY, deltaR;
    private boolean isAtTarget;

    private double errorX, errorY; //PEE JAY

    private BasicPID xController, yController, rController;
    private PIDCoefficients xyCoeff, rCoeff;

    private double xOut, yOut, rOut;
    private double twistedR, count, lastAngle;
    private double xPower, yPower;

    private boolean isAtXYTarget;

    public DT(HardwareMap hardwareMap){
        this.vs = hardwareMap.voltageSensor.iterator().next();
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        this.rightRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftRear.setDirection(DcMotor.Direction.REVERSE);
        this.xyCoeff = new PIDCoefficients(DTConstants.xyP, DTConstants.xyI, DTConstants.xyD);
        this.rCoeff = new PIDCoefficients(DTConstants.rP, DTConstants.rI, DTConstants.rD);
        this.xController = new BasicPID(xyCoeff);
        this.yController = new BasicPID(xyCoeff);
        this.rController = new BasicPID(rCoeff);
        this.xTarget = 0;
        this.yTarget = 0;
        this.rTarget = 0;
    }
    public DT(HardwareMap hardwareMap, Pose2d startPose){
        this.vs = hardwareMap.voltageSensor.iterator().next();
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
        this.xyCoeff = new PIDCoefficients(DTConstants.xyP, DTConstants.xyI, DTConstants.xyD);
        this.rCoeff = new PIDCoefficients(DTConstants.rP, DTConstants.rI, DTConstants.rD);
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
        yOut = -yController.calculate(yTarget, yRn);
        deltaY = yTarget - yRn; //PEE JAY
        deltaX = xTarget - xRn; //PEE JAY
//        errorX = deltaX * T.cos(rRn) - deltaY * T.sin(rRn); //PEE JAY
//        errorY = deltaX * T.sin(rRn) + deltaY * T.cos(rRn); //PEE JAY
        xOut = xController.calculate(xTarget, xRn); //PEE JAY
        yOut = -yController.calculate(yTarget, yRn); //PEE JAY
        if(Math.abs(rRn - lastAngle) > M.PI) count += Math.signum(lastAngle - rRn);
        lastAngle = rRn;
        twistedR = count * (2* M.PI) + rRn;
        rOut = -rController.calculate(rTarget, twistedR);
        xPower = xOut * T.cos(rRn) - yOut * T.sin(rRn);
        yPower = xOut * T.sin(rRn) + yOut * T.cos(rRn);

        deltaR = rTarget - twistedR;

        if(Math.abs(xPower) > DTConstants.maxAxialPower) xPower = DTConstants.maxAxialPower * Math.signum(xPower);
        if(Math.abs(yPower) > DTConstants.maxAxialPower) yPower = DTConstants.maxAxialPower * Math.signum(yPower);
        if(Math.abs(rOut) > DTConstants.maxAngularPower) rOut = DTConstants.maxAngularPower * Math.signum(rOut);

        if(Math.abs(xPower) < 0.1) xPower = 0;
        else xPower += DTConstants.XYBasePower * Math.signum(xPower);
        if(Math.abs(yPower) < 0.1) yPower = 0;
        else yPower += DTConstants.XYBasePower * Math.signum(yPower); // doesnt work since y becomes x and x becomes y
        if (Math.abs(rOut) < 0.05 || Math.abs(deltaR) < DTConstants.allowedAngularError) rOut = 0;
        else rOut += DTConstants.RBasePower * Math.signum(rOut); //this works tho
//        if((Math.abs(deltaX) < DTConstants.allowedAxialError) && (Math.abs(deltaY) < DTConstants.allowedAxialError)) {
//            isAtXYTarget = true;
//            xPower = 0;
//            yPower = 0;
//        } else {
//            isAtXYTarget = false;
//            xPower += DTConstants.XYBasePower * Math.signum(xPower);
//            yPower += DTConstants.XYBasePower * Math.signum(yPower);
//        }
        compensator = vs.getVoltage() / 12.5;
        xPower/=compensator;
        yPower/=compensator;
        rOut/=compensator;

        setPowers(xPower, yPower,rOut);
    }
    public Pose2d getLocation(){
        return new Pose2d(xRn, yRn, rRn);
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
        return rOut;
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
        return twistedR;
    }
    public double getYTarget(){
        return yTarget;
    }
    public double getUnTwistedR(){
        return rRn;
    }
    public double getDeltaX(){
        return deltaX;
    }
    public double getDeltaY(){
        return deltaY;
    }
    public double getDeltaR(){
        return deltaR;
    }
    public boolean isAtTarget(){
        return isAtTarget;
    }
    public boolean isAtTargetX(){
        return Math.abs(xRn - xTarget) < DTConstants.allowedAxialError;
    }
    public boolean isAtTargetY(){
        return Math.abs(yRn - yTarget) < DTConstants.allowedAxialError;
    }
    public boolean isAtTargetR(){
        return Math.abs(twistedR - rTarget) < DTConstants.allowedAngularError;
    }

    public double getTwistedR(){
        return 0;
    }

    public void lineTo(double x, double y, double r){
        setXTarget(x);
        setYTarget(y);
        setRTarget(r);
    }
    public void lineToCHeading(double x, double y){
        setXTarget(x);
        setYTarget(y);
    }
    public void lineToChangeHeadingUnderCondition(double x, double y, double r, boolean condition){
        setXTarget(x);
        setYTarget(y);
        if(condition){
            setRTarget(r);
        }
    }
}
