package org.firstinspires.ftc.teamcode.hardware.Deposit;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@Config
public class DepoArm {
    public enum DepoArmState {
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
    }

    public DepoArmState depoArmFSM = DepoArmState.STOPPED;


    private static final double LEFT_LOWER_BOUND = 0.1; //0
    private static final double LEFT_UPPER_BOUND = 1; //0.5
    private static final double RIGHT_LOWER_BOUND = 0.1;
    private static final double RIGHT_UPPER_BOUND = 1;

    private ServoMotorBetter leftArm;
    private ServoMotorBetter rightArm;

    public double target = 0;
    public double manualPosition = .5;
    public int level = 0;
    public double[] levelOffset = {.15,.2,.35,.4,.37,.5,.5};

    public DepoArm(ServoMotorBetter leftArm, ServoMotorBetter rightArm) {
        this.leftArm = leftArm;
        this.leftArm.setLowerBound(DepoArm.LEFT_LOWER_BOUND);
        this.leftArm.setUpperBound(DepoArm.LEFT_UPPER_BOUND);

        this.rightArm = rightArm;
        this.rightArm.setLowerBound(DepoArm.RIGHT_LOWER_BOUND);
        this.rightArm.setUpperBound(DepoArm.RIGHT_UPPER_BOUND);
    }
    public DepoArm.DepoArmState getState(){
        return depoArmFSM;
    }

    public void setManualPosition(double position){
        manualPosition = position;
    }



    public void setState(DepoArmState state){
        this.depoArmFSM = state;
        switch (depoArmFSM){
            case INITIALIZE:
                target = .85;//.1
                break;
            case TRANSFER:
                target = .84;
                break;
            case SCORE:
                target = 0 + levelOffset[level];
                break;
            case INTERMEDIATE:
                target = .6; //WRIST MUST ROTATE HERE
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
                target = 0.05;
                break;
            case Manual:
                target = manualPosition;
                break;
                //TODO add sanity check to make sure that it is between 0 and 1;
        }
    }

    public void update(){
        this.rightArm.setPosition(target);
        this.leftArm.setPosition(target);
        this.rightArm.update();
        this.leftArm.update();
    }
}
