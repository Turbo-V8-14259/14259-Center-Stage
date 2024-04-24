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

    private boolean purePersuiting = false;

    private BasicPID xController, yController, rController, pprController;
    private PIDCoefficients xyCoeff, rCoeff, pprCoeff;

    private double xOut, yOut, rOut;
    private double twistedR, count, lastAngle;
    private double xPower, yPower;

    private boolean isAtXYTarget;

    private double maxPower = 1;
    private double flPower, frPower, blPower, brPower;

    public DT(HardwareMap hardwareMap){
        this.vs = hardwareMap.voltageSensor.iterator().next();
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.drive.setPoseEstimate(new Pose2d(0, 0, 0));
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
        this.pprController = new BasicPID(pprCoeff);
        this.xTarget = 0;
        this.yTarget = 0;
        this.rTarget = 0;
        if(drive.getPoseEstimate().getX() != 0 || drive.getPoseEstimate().getY() != 0 || drive.getPoseEstimate().getHeading() != 0){
            drive.setPoseEstimate(new Pose2d(0, 0, 0));
        }
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
        this.pprController = new BasicPID(pprCoeff);

        this.xTarget = startPose.getX();
        this.yTarget = startPose.getY();
        this.rTarget = startPose.getHeading();
    }
    public void setPowers(double y, double x, double r){
        flPower = (y+x+r);
        blPower = (y-x+r);
        brPower = (y+x-r);
        frPower = (y-x-r);
        double normalize = Math.max(Math.abs(flPower) + Math.abs(blPower) + Math.abs(brPower) + Math.abs(frPower),1);
        leftFront.setPower((flPower/normalize) * maxPower);
        leftRear.setPower((blPower/normalize) * maxPower);
        rightRear.setPower((brPower/normalize) * maxPower);
        rightFront.setPower((frPower/normalize) * maxPower);
    }
    public void update(){
        drive.updatePoseEstimate();
        xRn = drive.getPoseEstimate().getX();
        yRn = drive.getPoseEstimate().getY();
        rRn = drive.getPoseEstimate().getHeading();
        xOut = xController.calculate(xTarget, xRn);
        yOut = -yController.calculate(yTarget, yRn);
        deltaY = yTarget - yRn;
        deltaX = xTarget - xRn;
//        xOut = xController.calculate(xTarget, xRn);
//        yOut = -yController.calculate(yTarget, yRn);

        if(Math.abs(rRn - lastAngle) > M.PI) count += Math.signum(lastAngle - rRn);
        lastAngle = rRn;
        twistedR = count * (2* M.PI) + rRn;

        if(purePersuiting){
            rOut = -pprController.calculate(rTarget, twistedR);
        }else{
            rOut = -rController.calculate(rTarget, twistedR);
        }
//        rOut = -rController.calculate(rTarget, twistedR);
        xPower = xOut * T.cos(rRn) - yOut * T.sin(rRn);
        yPower = xOut * T.sin(rRn) + yOut * T.cos(rRn);

        deltaR = rTarget - twistedR;

        if(Math.abs(xPower) > DTConstants.maxAxialPower) xPower = DTConstants.maxAxialPower * Math.signum(xPower);
        if(Math.abs(yPower) > DTConstants.maxAxialPower) yPower = DTConstants.maxAxialPower * Math.signum(yPower);
        if(Math.abs(rOut) > DTConstants.maxAngularPower) rOut = DTConstants.maxAngularPower * Math.signum(rOut);

        if(Math.abs(xPower) < 0.07) xPower = 0;
        else xPower += DTConstants.XYBasePower * Math.signum(xPower)* 1/maxPower;
        if(Math.abs(yPower) < 0.07) yPower = 0;
        else yPower += DTConstants.XYBasePower * Math.signum(yPower) * 1/maxPower;
        if (Math.abs(deltaR) < DTConstants.allowedAngularError) rOut = 0;
        else rOut += DTConstants.RBasePower * Math.signum(rOut) * 1/maxPower;
        compensator = vs.getVoltage() / 12.5;
        xPower/=compensator;
        yPower/=compensator;
        rOut/=compensator;

//        setPowers(xPower, yPower,rOut);
//        setPowers(-xPower, 0,0);
        setPowers(-xPower, -yPower,rOut);


        if(Math.abs(deltaX) < 2 && Math.abs(deltaY)<2 && Math.abs(deltaR) < DTConstants.allowedAngularError){
            isAtTarget = true;
        }else{
            isAtTarget = false;
        }
    }

    public void setPurePersuiting(boolean isPurePersuiting){
        purePersuiting = isPurePersuiting;
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
    public double getXTarget(){
        return xTarget;
    }
    public double getRTarget(){
        return rTarget;
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
    } //??
    public void setPoseEstimate(Pose2d pose){
        this.drive.setPoseEstimate(pose);
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
    public void setMaxPower(double maxPower){
        this.maxPower = maxPower;
    }

    //Takes in current posx, posy, posr, targetx, and targety
    //calculates the angle to follow that requires the least rotation
    public double toPoint(double x, double y, double r, double x2, double y2){
        double x3 = x2 - x;
        double y3 = y2 - y;

        double x4 = Math.cos(r);
        double y4 = Math.sin(r);

        double dotProduct = x3 * x4 + y3 * y4;

        double magnitude = Math.sqrt(x3 * x3 + y3 * y3);
        double cosine = Math.acos(dotProduct / magnitude);
        double angle = Math.atan2(y3, x3);
        double finalAngle=0;

        if(nearlyEqual(Math.cos(angle), Math.cos(r+cosine))&&nearlyEqual(Math.sin(angle), Math.sin(r+cosine))){
            finalAngle = r+cosine;
        }
        if(nearlyEqual(Math.cos(angle), Math.cos(r-cosine))&&nearlyEqual(Math.sin(angle), Math.sin(r-cosine))){
            finalAngle = r-cosine;
        }


        return finalAngle;
    }
    public boolean nearlyEqual(double a, double b) {
        return Math.abs(a - b) < 1e-9;
    }
}
