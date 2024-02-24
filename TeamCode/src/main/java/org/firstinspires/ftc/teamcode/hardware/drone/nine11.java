package org.firstinspires.ftc.teamcode.hardware.drone;

import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

public class nine11 {
    public enum DroneState {
        INITIALIZE,
        SCORE,
        CALCAULATE,
        STOPPED
    }
    public DroneState droneFSM = DroneState.STOPPED;

    public double target = 0;

    private ServoMotorBetter drone;
    private static final double LOWER_BOUND = 0.23;
    private static final double UPPER_BOUND = 0.55;

    public nine11(ServoMotorBetter drone){
        this.drone = drone;
        this.drone.setLowerBound(LOWER_BOUND);
        this.drone.setUpperBound(UPPER_BOUND);
    }

    public nine11.DroneState getState(){
        return droneFSM;
    }
    public void setState(DroneState state){
        this.droneFSM = state;
        switch(droneFSM){
            case INITIALIZE:
                target = 1;
                break;
            case SCORE:
                target = 0;
            case CALCAULATE:
                break;
            case STOPPED:
                break;
        }
    }
    public void update(){
        this.drone.setPosition(target);
        this.drone.update();
    }

}
