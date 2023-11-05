package org.firstinspires.ftc.teamcode.hardware.Deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Sensors.axonEncoder;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.AnglePID;


@Config
public class DepoTurret{
    public enum TurretState {
        TRANSFER,
        TELE_SCORING,
        AUTO_SCORING_ANGLE,
        AUTO_1,
        AUTO_2,
        AUTO_3,
        ANGLE_LOCK,
        STOPPED
    }
    public DepoTurret.TurretState turretFSM = TurretState.STOPPED;
    private int count = 0;
    private double currentAngle;
    private double lastAngle;
    public double lockTarget, robotAngle;
    public int[] presetAngle = {0, 180};
    public double target;

    public static double Kp = 0.006, Ki = 0.0025, Kd = 0;
    public AnglePID servoController;
    private CRServo turret;
    private AnalogInput angularEncoder;
    private axonEncoder angle;

    public boolean pidRunning = true;
    private double offset = 100;
    public DepoTurret(CRServo turret, AnalogInput angularEncoder) {
        this.turret = turret;
        this.angularEncoder = angularEncoder;
        this.angle = new axonEncoder(angularEncoder);
        this.servoController = new AnglePID(new AnglePID.Coefficients(Kp, Ki, Kd),
                () -> this.updateAngle() - this.setTargetAngle(), //what the fu

                factor -> this.turret.setPower(M.clamp(factor, -.5, .5)));
    }
    public double setTargetAngle(){
        return target;
    }
    public DepoTurret.TurretState getState(){
        return turretFSM;
    }//untested
    public double updateAngle(){
        currentAngle = angle.update() + offset;
        if(Math.abs(currentAngle - lastAngle) > 180){
            count += Math.signum(lastAngle - currentAngle);
        }
        lastAngle = currentAngle;
        return count * 360 + currentAngle;
    }
    public void angleLockSet(double lockTarg, double robAng){
        this.lockTarget = lockTarg;
        this.robotAngle = robAng;
    }
    public void angleLock(){
        this.target = lockTarget - robotAngle;
    }
    public void setState(DepoTurret.TurretState state){
        this.turretFSM = state;
        switch(turretFSM){
            case TRANSFER:
                pidRunning = true;
                this.target = presetAngle[0];
                break;
            case TELE_SCORING:
                pidRunning = true;
                this.target = presetAngle[1];
                break;
            case AUTO_SCORING_ANGLE:
                break;
            case AUTO_1:
                break;
            case ANGLE_LOCK:
                pidRunning = true;
                this.angleLock();
                break;
            case STOPPED:
                this.pidRunning = false;
                break;
        }//untested
    }
    public void update() {
        if(pidRunning){
            this.target = setTargetAngle();
            this.servoController.update();
        }else{
            //lol idk
        }
    }
}
