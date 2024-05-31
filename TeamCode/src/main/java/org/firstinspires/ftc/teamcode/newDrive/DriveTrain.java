package org.firstinspires.ftc.teamcode.newDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;

public class DriveTrain {
    private SampleMecanumDrive drive;
    private DcMotorEx leftFront, rightRear, rightFront, leftRear;
    private boolean forceStop = false;
    private boolean voltageCompensating;
    private ElapsedTime timer;
    private VoltageSensor vs;
    private double compensator;
    private double twistedR, count, lastAngle, rRn;
    private Pose2d currentPos;

    //heading calculations
    public DriveTrain(HardwareMap hardwareMap, Pose2d startPose){
        this.drive = new SampleMecanumDrive(hardwareMap);
        //set our start pose
        this.drive.setPoseEstimate(startPose);
        this.leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        //motor directions
        this.rightRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftRear.setDirection(DcMotor.Direction.REVERSE);
    }

    //overloaded constructor for voltage compensation
    public DriveTrain(HardwareMap hardwareMap, Pose2d startPose, ElapsedTime timer){
        this.drive = new SampleMecanumDrive(hardwareMap);
        //set our start pose
        this.drive.setPoseEstimate(startPose);
        this.leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        this.leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        this.rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
        this.rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");
        //motor directions
        this.rightRear.setDirection(DcMotor.Direction.FORWARD);
        this.rightFront.setDirection(DcMotor.Direction.FORWARD);
        this.leftFront.setDirection(DcMotor.Direction.REVERSE);
        this.leftRear.setDirection(DcMotor.Direction.REVERSE);
        this.voltageCompensating = true;
        this.vs = hardwareMap.voltageSensor.iterator().next();
        this.timer = timer;
    }

    public void setPowers(double y, double x, double r){
        if(!forceStop){
            double max = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
            if(!voltageCompensating) {
                double flPower = (y + x + r);
                double blPower = (y - x + r);
                double brPower = (y + x - r);
                double frPower = (y - x - r);
                leftFront.setPower((flPower / max));
                leftRear.setPower((blPower / max));
                rightRear.setPower((brPower / max));
                rightFront.setPower((frPower / max));
            }else{
                //only need to read the voltage sensor once in a while (5 seconds).
                if(timer.milliseconds() % 5000 == 0){
                    compensator = vs.getVoltage() / 12.5;
                }
                double flPower = (y + x + r);
                double blPower = (y - x + r);
                double brPower = (y + x - r);
                double frPower = (y - x - r);
                leftFront.setPower((flPower / max)/compensator);
                leftRear.setPower((blPower / max)/compensator);
                rightRear.setPower((brPower / max)/compensator);
                rightFront.setPower((frPower / max)/compensator);
            }
        }else{
            //motors at powers 0 if force stop is true
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }

    public void updatePos(){ //updates our current global position, heading is -inf to +inf
        drive.updatePoseEstimate();
        currentPos = drive.getPoseEstimate();
        rRn = currentPos.getHeading();
        if(Math.abs(rRn - lastAngle) > M.PI) count += Math.signum(lastAngle - rRn);
        twistedR = count * (2* M.PI) + rRn;
        lastAngle = rRn;
    }
    public Pose2d getGlobalPos(){ //returns our current global position
        return new Pose2d(currentPos.getX(), currentPos.getY(), twistedR);
    }

    public double getrRn(){
        return rRn;
    }
}
