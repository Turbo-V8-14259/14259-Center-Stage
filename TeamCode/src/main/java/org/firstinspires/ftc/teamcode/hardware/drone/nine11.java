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

    public nine11.DroneState getState(){
        return droneFSM;
    }
    public void setState(DroneState state){
        this.droneFSM = state;
        switch(droneFSM){
            case INITIALIZE:
                target = 0;
                break;
            case SCORE:
                target = 1;
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
