package org.firstinspires.ftc.teamcode.hardware.Deposit;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@Config
public class DepoArm {
    public enum DepoArmState {
        INITIALIZE,
        TRANSFER,
        SCORE,
        PITCH_AT_MID_INTERMEDIATE,
        PITCH_AT_MID_SCORING,
        INTERMEDIATE,
        STOPPED
    }

    public DepoArmState depoArmFSM = DepoArmState.STOPPED;

    private static final double LEFT_LOWER_BOUND = 0;
    private static final double LEFT_UPPER_BOUND = 0.5;
    private static final double RIGHT_LOWER_BOUND = 0;
    private static final double RIGHT_UPPER_BOUND = 1;

    private ServoMotorBetter leftArm;
    private ServoMotorBetter rightArm;

    public double target = 0;

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



    public void setState(DepoArmState state){
        this.depoArmFSM = state;
        switch (depoArmFSM){
            case INITIALIZE:
                target = 0;//.1
                break;
            case TRANSFER:
                target = 0;
                break;
            case SCORE:
                target = .35;
                break;
            case INTERMEDIATE:
                target = 0.6;
                break;
            case STOPPED:
                break;
        }
    }

    public void update(){
        this.rightArm.setPosition(target);
        this.leftArm.setPosition(target);
        this.rightArm.update();
        this.leftArm.update();
    }
}
