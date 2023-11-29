package org.firstinspires.ftc.teamcode.hardware.Deposit;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@Config
public class LM1Turret {
    public enum TurretState {
        INITIALIZE,
        SCORE,
        PITCH_MID,
        STOPPED,

        AUTOLOCK,

        RUNTOPOSITION,
    }

    public double robotAngle = 0;
    public double manualPosition;

    public TurretState turretFSM = TurretState.STOPPED;

    private static final double LEFT_LOWER_BOUND = 1;
    private static final double LEFT_UPPER_BOUND = .14;

    private static final double LEFT_90_DEGREES = 0.86;

    private static final double RIGHT_90_DEGREES = 0.42;
    // temp number, will tune today

    private ServoMotorBetter turret;

    public double target = 0;

    public LM1Turret(ServoMotorBetter leftArm) {
        this.turret = leftArm;
        this.turret.setLowerBound(LM1Turret.LEFT_LOWER_BOUND);
        this.turret.setUpperBound(LM1Turret.LEFT_UPPER_BOUND);
    }
    public LM1Turret.TurretState getState(){
        return turretFSM;
    }

    public double calculateRotation(double robotA){
        double calculatedAngle = (2*robotA/3.1415) * (LEFT_90_DEGREES - RIGHT_90_DEGREES) + 0.65;
        return calculatedAngle > LEFT_90_DEGREES || calculatedAngle < RIGHT_90_DEGREES ? 0.65 : calculatedAngle;

    }
    public void autoLock(){


    }
    public void setState(TurretState state){
        this.turretFSM = state;
        switch (turretFSM){
            case INITIALIZE:
                target = 0.01;
                break;
            case SCORE:
                target = 0.65;
            case STOPPED:
                break;
            case AUTOLOCK:
                target = calculateRotation(robotAngle);
                break;
            case RUNTOPOSITION:
                target = manualPosition;
        }
    }

    public void update(){
        this.turret.setPosition(target);
        this.turret.update();
    }


}
