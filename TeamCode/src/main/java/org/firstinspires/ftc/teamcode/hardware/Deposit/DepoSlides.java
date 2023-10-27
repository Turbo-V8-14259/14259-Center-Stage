package org.firstinspires.ftc.teamcode.hardware.Deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;


@Config
public class DepoSlides{

    public enum DepositState {
        UP,
        MIDDLE,
        DOWN,
        STOPPED
    }

    public DepositState depositFSM;

    public double target;
    public double targetDepositInches;

    private static final double INCHES_TO_TICKS = 100; //number
    private static final double LOWER_BOUND = 0;
    private static final double UPPER_BOUND = 2600; //number
    private static final double INIT_INCHES = 0;

    private double targetLinSlidePosition = 0;

    private DcMotorBetter leftMotor;
    private DcMotorBetter rightMotor;

    private PID linSlideController;

    public static double Kp = 1, Ki = 0, Kd = 0;

    public boolean pidRunning = true;

    //    public double passivePower = 0;
    //this should contain a power that holds the slides up when its not moving; you probably need to use trig for this since the slides change angle.


    public DepoSlides(DcMotorBetter leftMotor, DcMotorBetter rightMotor) {
        this.leftMotor = leftMotor;
        this.leftMotor.setLowerBound(DepoSlides.LOWER_BOUND);
        this.leftMotor.setUpperBound(DepoSlides.UPPER_BOUND);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.rightMotor = rightMotor;
        this.rightMotor.setLowerBound(DepoSlides.LOWER_BOUND);
        this.rightMotor.setUpperBound(DepoSlides.UPPER_BOUND);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.linSlideController = new PID(new PID.Coefficients(Kp, Ki, Kd),
                () -> this.leftMotor.getCurrentPosition()-this.targetLinSlidePosition,
                factor -> {
                    this.leftMotor.setPower(M.clamp(-factor, 1, -1));
                    this.rightMotor.setPower(M.clamp(-factor, 1, -1));
                });
    }

    public void setInches(double inches) {
        this.target = (M.normalize((inches - DepoSlides.INIT_INCHES) * DepoSlides.INCHES_TO_TICKS, DepoSlides.LOWER_BOUND, DepoSlides.UPPER_BOUND));
    }
    public double getCurrentInches() {
        return M.lerp(DepoSlides.LOWER_BOUND, DepoSlides.UPPER_BOUND, this.getCurrentPosition()) / DepoSlides.INCHES_TO_TICKS + DepoSlides.INIT_INCHES;
    }

    public DepoSlides setPower(double power) {
        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
        return this;
    }

    public DepoSlides stop() {
        this.leftMotor.stop();
        this.rightMotor.stop();
        return this;
    }

    public DepoSlides stopAndResetEncoder() {
        this.leftMotor.stopAndResetEncoder();
        this.rightMotor.stopAndResetEncoder();
        return this;
    }

    public boolean isBusy() { return this.leftMotor.isBusy() || this.rightMotor.isBusy(); }

    public double getCurrentPosition() {
        return this.leftMotor.getCurrentPosition();
    }

    public double setTargetLinSlidePosition(){
        return target;
    }

    public void update() {
        switch(depositFSM){
            case UP:
                this.targetDepositInches = 15; //placehollder
                this.setInches(targetDepositInches);
                break;
            case MIDDLE:
                break;
            case DOWN:
                break;
            case STOPPED:
                this.pidRunning = false;
                break;
        }
        this.targetLinSlidePosition = setTargetLinSlidePosition();
        this.linSlideController.update();
        this.leftMotor.update();
        this.rightMotor.update();
    }


    //    public double getTargetInches() {
    //        return M.lerp(DepoSlides.LOWER_BOUND, DepoSlides.UPPER_BOUND, this.getTargetPosition()) / DepoSlides.INCHES_TO_TICKS + DepoSlides.INIT_INCHES;
    //    }

    //    public double getTargetPosition() {
    //        return this.leftMotor.getTargetPosition();
    //    }

    //    public DepoSlides addPosition(double position) {
    //        this.leftMotor.addPosition(position);
    //        this.rightMotor.addPosition(position);
    //        return this;
    //    }
    //
    //    public DepoSlides addPower(double power) {
    //        this.leftMotor.addPower(power);
    //        this.rightMotor.addPower(power);
    //        return this;
    //    }


    //    public DepoSlides setPosition(double position) {
    //        this.leftMotor.setPosition(position);
    //        this.rightMotor.setPosition(position);
    //        return this;
    //    }
}
