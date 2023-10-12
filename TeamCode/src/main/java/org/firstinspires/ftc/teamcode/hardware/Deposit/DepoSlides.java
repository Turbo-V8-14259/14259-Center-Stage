package org.firstinspires.ftc.teamcode.hardware.Deposit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;

public class DepoSlides{
    public double target;
    private static final double INCHES_TO_TICKS = 0; //number
    private static final double LOWER_BOUND = 0;
    private static final double UPPER_BOUND = 0; //number
    private static final double INIT_INCHES = 0;

    private double targetLinSlidePosition = 0;

    public DcMotorBetter leftMotor;
    public DcMotorBetter rightMotor;

    public PID linSlideController;

    private double Kp = 0, Ki = 0, Kd = 0;


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
                () -> this.targetLinSlidePosition - this.leftMotor.getCurrentPosition(),
                factor -> {
                    this.leftMotor.setPower(M.clamp(-factor, .5, -.5));
                    this.rightMotor.setPower(M.clamp(-factor, .5, -.5));
                });
    }

    public DepoSlides setPosition(double position) {
        this.leftMotor.setPosition(position);
        this.rightMotor.setPosition(position);
        return this;
    }

    public DepoSlides setInches(double inches) {
        this.setPosition(M.normalize((inches - DepoSlides.INIT_INCHES) * DepoSlides.INCHES_TO_TICKS, DepoSlides.LOWER_BOUND, DepoSlides.UPPER_BOUND));
        return this;
    }

    public DepoSlides setPower(double power) {
        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
        return this;
    }

    public DepoSlides addPosition(double position) {
        this.leftMotor.addPosition(position);
        this.rightMotor.addPosition(position);
        return this;
    }

    public DepoSlides addPower(double power) {
        this.leftMotor.addPower(power);
        this.rightMotor.addPower(power);
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

    public double getTargetPosition() {
        return this.leftMotor.getTargetPosition();
    }

    public double getCurrentInches() {
        return M.lerp(DepoSlides.LOWER_BOUND, DepoSlides.UPPER_BOUND, this.getCurrentPosition()) / DepoSlides.INCHES_TO_TICKS + DepoSlides.INIT_INCHES;
    }

    public double getTargetInches() {
        return M.lerp(DepoSlides.LOWER_BOUND, DepoSlides.UPPER_BOUND, this.getTargetPosition()) / DepoSlides.INCHES_TO_TICKS + DepoSlides.INIT_INCHES;
    }

    public double setTargetLinSlidePosition(){
        return target;
    }

    public void update() {
        this.targetLinSlidePosition = setTargetLinSlidePosition();
        this.linSlideController.update();
        this.leftMotor.update();
        this.rightMotor.update();
    }
}
