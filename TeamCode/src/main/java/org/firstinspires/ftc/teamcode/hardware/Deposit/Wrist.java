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
        LT_SCORE
    }

    public WristState wristFSM = WristState.STOPPED;
    private static final double LEFT_LOWER_BOUND = 0; //0
    private static final double LEFT_UPPER_BOUND = 1; //0.5

    private ServoMotorBetter wrist;

    public double target = 0;
    public double manualPosition = 0;
    public int level = 0;
    public double[] levelOffset = {.15,.2,.35,.4,.37,.5,.5};

    public Wrist(ServoMotorBetter wrist) {
        this.wrist = wrist;
        this.wrist.setLowerBound(LEFT_LOWER_BOUND);
        this.wrist.setUpperBound(LEFT_UPPER_BOUND);
    }
    public Wrist.WristState getState(){
        return wristFSM;
    }



    public void setState(WristState state){
        this.wristFSM = state;
        switch (wristFSM){
            case INITIALIZE:
                target = 0;//.1
                break;
            case TRANSFER:
                target = 0;
                break;
            case SCORE:
                target = 1 - levelOffset[level];
                break;
            case INTERMEDIATE:
                target = 1;
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
            case LT_SCORE:
                target = 1;
                //TODO add sanity check to make sure that it is between 0 and 1;
        }
    }

    public void update(){
        this.wrist.setPosition(target);
        this.wrist.update();
    }
}
