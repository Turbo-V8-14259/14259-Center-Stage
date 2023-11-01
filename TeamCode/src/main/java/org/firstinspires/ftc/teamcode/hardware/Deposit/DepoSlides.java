package org.firstinspires.ftc.teamcode.hardware.Deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;


@Config
public class DepoSlides{

    public int[] presetInches = {0, 15, 30, 45};

    public enum DepositState {
        UP,
        MIDDLE,
        DOWN,
        AUTO_EXTENSION,
        STOPPED
    }

    public DepositState depositFSM = DepositState.STOPPED;

    public double target;
    public double maxTargetInches;
    public double targetDepositInches;

    private static final double INCHES_TO_TICKS = 100; //number
    private static final double LOWER_BOUND = 0; //number in ticks
    private static final double UPPER_BOUND = 2600; //number in ticks
    private static final double INIT_INCHES = 0;

    private double targetLinSlidePosition = 0;

    public DcMotorBetter leftMotor;
    private DcMotorBetter rightMotor;
    private PID linSlideController;

    public static double Kp = 2, Ki = 0, Kd = 0;


    //todo: pid running condition
    public boolean pidRunning = true;
    public boolean passive = false;
    public boolean manualMode = false;
    public double passivePower = 0.1;
    //this should contain a power that holds the slides up when its not moving; you probably need to use trig for this since the slides change angle.
    private double manualPower = 0;

    public DepoSlides(DcMotorBetter leftMotor, DcMotorBetter rightMotor) {
        this.leftMotor = leftMotor;
        this.leftMotor.setLowerBound(DepoSlides.LOWER_BOUND);
        this.leftMotor.setUpperBound(DepoSlides.UPPER_BOUND);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.rightMotor = rightMotor;
        this.rightMotor.setLowerBound(DepoSlides.LOWER_BOUND);
        this.rightMotor.setUpperBound(DepoSlides.UPPER_BOUND);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.linSlideController = new PID(new PID.Coefficients(Kp, Ki, Kd),
                () -> this.leftMotor.getCurrentPosition() - this.targetLinSlidePosition,
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
    public double getTargetInches(){
        return targetDepositInches;
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
    public void setState(DepositState state){
        this.depositFSM = state;
        switch(depositFSM){
            case UP:
                this.pidRunning = true;
                this.setInches(15);
                break;
            case MIDDLE:
                this.pidRunning = true;
                this.targetDepositInches = (this.maxTargetInches/2);
                this.setInches(targetDepositInches);
                break;
            case DOWN:
                this.pidRunning = true;
                this.targetDepositInches = presetInches[0];
                this.setInches(targetDepositInches);
                break;
            case AUTO_EXTENSION:
                break;
            case STOPPED:
                this.pidRunning = false;
                this.passive = true;
                break;
        }//untested
    }

    public DepoSlides.DepositState getState(){
        return depositFSM;
    }//untested
    public void setPowerManual(double power){
        this.manualPower = power;
    }


    public void update() {

        if(pidRunning){
            this.targetLinSlidePosition = setTargetLinSlidePosition();
            this.linSlideController.update();
        }else if(!pidRunning && passive){
            this.leftMotor.setPower(passivePower);
            this.rightMotor.setPower(passivePower);

        }else if(manualMode){
            this.leftMotor.setPower(manualPower);
            this.rightMotor.setPower(manualPower);
        else{
            this.leftMotor.setPower(0);
            this.rightMotor.setPower(0);
        }
        this.leftMotor.update();
        this.rightMotor.update();
    }
}
