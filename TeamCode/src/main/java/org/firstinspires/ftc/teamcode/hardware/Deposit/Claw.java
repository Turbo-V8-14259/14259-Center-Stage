package org.firstinspires.ftc.teamcode.hardware.Deposit;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@Config
public class Claw {
    public enum ClawState {
        LATCHED,
        UNLATCHED,
        INTAKE
    }

    public ClawState clawFSM = ClawState.LATCHED;


    private static final double LOWER_BOUND = .35;
    private static final double UPPER_BOUND = .44;


    private ServoMotorBetter claw;

    public double target = 0;

    public Claw(ServoMotorBetter claw) {
        this.claw = claw;
        this.claw.setLowerBound(UPPER_BOUND);
        this.claw.setUpperBound(LOWER_BOUND);
    }
    public Claw.ClawState getState(){
        return clawFSM;
    }



    public void setState(ClawState state){
        this.clawFSM = state;
        switch (clawFSM){
            case LATCHED:
                target = .75;
                break;
            case INTAKE:
                target = 0;
                break;
            case UNLATCHED:
                target = 0;
                break;
        }
    }

    public void update(){
        this.claw.setPosition(target);
        this.claw.update();
    }
}
