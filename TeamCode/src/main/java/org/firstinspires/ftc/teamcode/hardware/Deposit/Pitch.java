package org.firstinspires.ftc.teamcode.hardware.Deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;


@Config
public class Pitch {
    public enum PitchState {
        INITIALIZE,
        SCORE_MIDDLE,
        CLIMB,
        STOPPED,
        SCOREATLEVEL
    }

    public int level = 0;

    public double[] levels = {0,0.25,0.35,0.4,0.45,0.55};

    public PitchState pitchFSM = PitchState.STOPPED;

    public double target = 0;
    private static final double DEGREES_TO_TICKS = 0;
    private static final double LOWER_BOUND = 0;
    private static final double UPPER_BOUND = 7250;
    private static final double INIT_DEGREES = 0;

    private double targetPitchPosition = 0;

    public DcMotorBetter pitchMotor;

    public PID pitchController;

    public static double Kp = 22, Ki = 0.03, Kd = 0;

    public boolean manualMode = false;

    private double manualPower = 0;

    public Pitch(DcMotorBetter pitchMotor) {
        this.pitchMotor = pitchMotor;
        this.pitchMotor.setLowerBound(Pitch.LOWER_BOUND);
        this.pitchMotor.setUpperBound(Pitch.UPPER_BOUND);
        this.pitchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.pitchController = new PID(new PID.Coefficients(Kp, Ki, Kd),
                () -> this.pitchMotor.getCurrentPosition() - this.targetPitchPosition,
                factor -> this.pitchMotor.setPower(M.clamp(-factor, 1, -1)));
    }

    public void setDegrees(double deg) {
        this.target = (M.normalize((deg - Pitch.INIT_DEGREES) * Pitch.DEGREES_TO_TICKS, Pitch.LOWER_BOUND, Pitch.UPPER_BOUND));
    }
    //"its not linear :nerd:" - Leo

    public double getCurrentDegrees() {
        return M.lerp(Pitch.LOWER_BOUND, Pitch.UPPER_BOUND, this.getCurrentPosition()) / Pitch.DEGREES_TO_TICKS + Pitch.INIT_DEGREES;
    }
    //"its not linear :nerd:" - Leo

    public Pitch setPower(double power) {
        this.pitchMotor.setPower(power);
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

    public void setPowerManual(double power){
        this.manualPower = power;
    }

    public double setTargetPitchPosition() {
        return target;
    }

    public Pitch.PitchState getState(){
        return pitchFSM;
    }

    public void setState(PitchState state){
        this.pitchFSM = state;
        switch (pitchFSM){
            case INITIALIZE:
                target = 0;
                break;
            case SCORE_MIDDLE:
                target = 0.2;
                break;
            case CLIMB:
                target = 1;
                break;
            case STOPPED:
                break;
            case SCOREATLEVEL:
                target = levels[level];
                break;
        }
    }

    public void update() {
        if(manualMode){
            this.pitchMotor.setPower(manualPower);
        }else{
            this.targetPitchPosition = setTargetPitchPosition();
            this.pitchController.update();
        }
        this.pitchMotor.update();
    }
}
