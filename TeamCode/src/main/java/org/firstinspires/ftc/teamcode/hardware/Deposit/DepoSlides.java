package org.firstinspires.ftc.teamcode.hardware.Deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;


@Config
public class DepoSlides {
    public enum DepositState {
        UP,
        DOWN,
        STOPPED,
        INTERMEDIATE_INCRIMENT,
        INCREMENT_UP,
        INCREMENT_DOWN,

        CALCULATED_UP,
        RUNTOPOSITION,
    }
    public DepositState depositFSM = DepositState.STOPPED;

    public double maxTargetInches = 35;
    private static final double INCHES_TO_TICKS = 34.848; //number
    private static final double LOWER_BOUND = 0; //number in ticks
    private static final double UPPER_BOUND = 1150; //number in ticks
    private static final double INIT_INCHES = 0;
    private double targetLinSlidePosition = 0;
    public double targetDepositInches = 0;
    public double target;
    public int[] presetInches = {0, 15, 30, (int) this.maxTargetInches};

    public int level = 0;
    public double manualPosition = 0;
    public int extension = 0;
    public double[] levels = {0,0.1,0.2,0.3,0.4,0.5};
    public double[] extensions = {0,0.2,0.4,0.6,0.8,1};


    public DcMotorBetter leftMotor;
    public DcMotorBetter rightMotor;
    private PID linSlideController;

    public static double Kp = 15, Ki = 0.01, Kd = 0; //NICE PID COEFFICIENT BRO

    public boolean pidRunning = true;
    public boolean passive = false;
    public boolean manualMode = false;
    public double passivePower = 0.1;
    //this should contain a power that holds the slides up when its not moving; you probably need to use trig for this since the slides change angle.
    private double manualPower = 0;

    public boolean isAtTarget = false;

    public DepoSlides(DcMotorBetter leftMotor, DcMotorBetter rightMotor) {
        this.leftMotor = leftMotor;
        this.leftMotor.setLowerBound(DepoSlides.LOWER_BOUND);
        this.leftMotor.setUpperBound(DepoSlides.UPPER_BOUND);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightMotor = rightMotor;
        this.rightMotor.setLowerBound(DepoSlides.LOWER_BOUND);
        this.rightMotor.setUpperBound(DepoSlides.UPPER_BOUND);
        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setDirection(DcMotor.Direction.REVERSE);
        this.linSlideController = new PID(new PID.Coefficients(Kp, Ki, Kd),
                () -> (this.leftMotor.getCurrentPosition()*-1) - this.targetLinSlidePosition,
                factor -> {
                    this.leftMotor.setPower(M.clamp(factor, 1, -1));
                    this.rightMotor.setPower(M.clamp(-factor, 1, -1));
                });
    }
    public void setInches(double inches) {
        this.target = -(M.normalize((inches - DepoSlides.INIT_INCHES) * DepoSlides.INCHES_TO_TICKS, DepoSlides.LOWER_BOUND, DepoSlides.UPPER_BOUND));
    }
    public double getCurrentInches() {
        return -M.lerp(DepoSlides.LOWER_BOUND, DepoSlides.UPPER_BOUND, this.getCurrentPosition()) / DepoSlides.INCHES_TO_TICKS + DepoSlides.INIT_INCHES;
    }
    public double getTargetInches() {
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
    public boolean isBusy() {
        return this.leftMotor.isBusy() || this.rightMotor.isBusy();
    }
    public double getCurrentPosition() {
        return this.leftMotor.getCurrentPosition();
    }
    public double setTargetLinSlidePosition() {
        return target;
    }
    public void setState(DepositState state) {
        this.depositFSM = state;
        switch (depositFSM) {
            case UP:
                this.targetDepositInches = 27;
                this.setInches(targetDepositInches);
                this.pidRunning = true;
                break;
            case DOWN:
                this.targetDepositInches = 0;
                this.setInches(targetDepositInches);
                this.pidRunning = true;
                break;
            case STOPPED:
                this.pidRunning = false;
                this.passive = true;
                break;
            case INTERMEDIATE_INCRIMENT:
                break;
            case CALCULATED_UP:
                this.targetDepositInches = calculateExtension();
                this.setInches(targetDepositInches);
                this.pidRunning = true;
                break;
            case RUNTOPOSITION:
                this.targetDepositInches = this.manualPosition;
                this.setInches(targetDepositInches);
                this.pidRunning = true;

        }//untested
    }
    public DepoSlides.DepositState getState() {
        return depositFSM;
    }
    public void setPowerManual(double power) {
        this.manualPower = power;
    }
    public void update() {
        if(pidRunning) {
            this.targetLinSlidePosition = setTargetLinSlidePosition();
            this.linSlideController.update();
        }else if (!pidRunning && passive) {
            this.leftMotor.setPower(passivePower);
            this.rightMotor.setPower(passivePower);
        }else if (!pidRunning && !passive && manualMode) {
            this.leftMotor.setPower(manualPower);
            this.rightMotor.setPower(-manualPower);
        }else{
            this.stop();
        }
        this.leftMotor.update();
        this.rightMotor.update();
        if (Math.abs(getCurrentPosition()-target) < 0.01){
            isAtTarget = true;
        }else{
            isAtTarget = false;
        }
    }
    public double calculateExtension(){ //returns inches
        double toInches =  (maxTargetInches*(levels[level])) + (maxTargetInches*(extensions[extension]));
        return Math.min(toInches, maxTargetInches);
    }
}

