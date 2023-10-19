package org.firstinspires.ftc.teamcode.hardware.Deposit;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;

public class Pitch {
    public double target = 0;
    private static final double RADIANS_TO_TICKS = 0;
    private static final double LOWER_BOUND = 0;
    private static final double UPPER_BOUND = 0;
    private static final double INIT_RADIANS = 0;

    private double targetPitchPosition = 0;

    public DcMotorBetter pitchMotor;

    public PID pitchController;

    private double Kp = 0, Ki = 0, Kd = 0;

    public Pitch(DcMotorBetter pitchMotor) {
        this.pitchMotor = pitchMotor;
        this.pitchMotor.setLowerBound(Pitch.LOWER_BOUND);
        this.pitchMotor.setUpperBound(Pitch.UPPER_BOUND);
        this.pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.pitchController = new PID(new PID.Coefficients(Kp, Ki, Kd),
                () -> this.targetPitchPosition - this.pitchMotor.getCurrentPosition(),
                factor -> this.pitchMotor.setPower(M.clamp(-factor, 0.5, -0.5)));
    }

    public Pitch setPosition(double position) {
        this.pitchMotor.setPosition(position);
        return this;
    }

    public Pitch setInches(double inches) {
        this.setPosition(M.normalize((inches - Pitch.INIT_RADIANS) * Pitch.RADIANS_TO_TICKS, Pitch.LOWER_BOUND, Pitch.UPPER_BOUND));
        return this;
    }

    public Pitch setPower(double power) {
        this.pitchMotor.setPower(power);
        return this;
    }

    public Pitch addPosition(double position) {
        this.pitchMotor.addPosition(position);
        return this;
    }

    public Pitch addPower(double power) {
        this.pitchMotor.addPower(power);
        return this;
    }

    public Pitch stop() {
        this.pitchMotor.stop();
        return this;
    }

    public Pitch stopAndResetEncoder() {
        this.pitchMotor.stopAndResetEncoder();
        return this;
    }

    public boolean isBusy() {
        return this.pitchMotor.isBusy();
    }

    public double getCurrentPosition() {
        return this.pitchMotor.getCurrentPosition();
    }

    public double getTargetPosition() {
        return this.pitchMotor.getTargetPosition();
    }

    public double getCurrentRadians() {
        return M.lerp(Pitch.LOWER_BOUND, Pitch.UPPER_BOUND, this.getCurrentPosition()) / Pitch.RADIANS_TO_TICKS + Pitch.INIT_RADIANS;
    }

    public double getTargetRadians() {
        return M.lerp(Pitch.LOWER_BOUND, Pitch.UPPER_BOUND, this.getTargetPosition()) / Pitch.RADIANS_TO_TICKS + Pitch.INIT_RADIANS;
    }

    public double setTargetPitchPosition() {
        return target;
    }

    public void update() {
        this.targetPitchPosition = setTargetPitchPosition();
        this.pitchController.update();
        this.pitchMotor.update();
    }
    //Aint this just my deposit code Leo
    //me when you didn't bother to remove the Inches functions for the pitch after copying the deposit :yaw:

}
