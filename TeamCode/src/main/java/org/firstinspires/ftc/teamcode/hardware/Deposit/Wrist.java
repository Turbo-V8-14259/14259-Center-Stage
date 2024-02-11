package org.firstinspires.ftc.teamcode.hardware.Deposit;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@Config
public class Wrist {
    public enum WristState {
        INITIALIZE,
        TRANSFER,
        SCORE,

        ABSOLUTE_INTERMEDIATE,

        INTERMEDIATE,
        STOPPED,
        RUNTOPOSITION,
        AUTO_PRELOAD,
        LT_SCORE,
        Manual,
        ABOVE_TRANSFER,
        CLIMB

    }

    public WristState wristFSM = WristState.STOPPED;
    private static final double LEFT_LOWER_BOUND = 0; //0
    private static final double LEFT_UPPER_BOUND = 1; //0.5

    private ServoMotorBetter wrist;

    public double target = 0;
    public double manualPosition = 0.5;
    public int level = 0;
    public double[] levelOffset = {0.1,0.08,0.04,0,0,0,.1};

    public Wrist(ServoMotorBetter wrist) {
        this.wrist = wrist;
        this.wrist.setLowerBound(LEFT_LOWER_BOUND);
        this.wrist.setUpperBound(LEFT_UPPER_BOUND);
    }
    public Wrist.WristState getState(){
        return wristFSM;
    }

    public void setLevel(int level){
        this.level = level;
    }

    public int getLevel(){
        return this.level;
    }

    public void setManualPosition(double position){
        manualPosition = position;
    }

    public void setState(WristState state){
        this.wristFSM = state;
        switch (wristFSM){
            case INITIALIZE:
                target = 0.35;//.1
                break;
            case TRANSFER:
                target = 0.0;
                break;
            case SCORE:
                target = 1 - levelOffset[level];
                break;
            case INTERMEDIATE:
                target = .55; //when arm is at 0.6 wrist must rotate
                break;
            case ABSOLUTE_INTERMEDIATE:
                target = 1;
            case STOPPED:
                break;
            case RUNTOPOSITION:
                target = manualPosition;
                break;
            case AUTO_PRELOAD:
                target = .95;
                break;
            case LT_SCORE:
                target = 1 - levelOffset[level];
                break;
            case Manual:
                target = manualPosition;
                break;
            case ABOVE_TRANSFER:
                target = 0.03;
                break;
            case CLIMB:
                target = 0.4;
                break;

                //TODO add sanity check to make sure that it is between 0 and 1;
        }
    }

    public void update(){
        this.wrist.setPosition(target);
        this.wrist.update();
    }
}
