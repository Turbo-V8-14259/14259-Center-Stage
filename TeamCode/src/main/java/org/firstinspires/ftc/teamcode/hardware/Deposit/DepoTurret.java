package org.firstinspires.ftc.teamcode.hardware.Deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Sensors.axonEncoder;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.AnglePID;


@Config
public class DepoTurret{
    public double target;

    int count = 0;
    double currentAngle;
    double lastAngle;

    public AnglePID servoController;
    CRServo turret;
    AnalogInput angularEncoder;
    axonEncoder angle;


    public static double Kp = 0.006, Ki = 0.0025, Kd = 0;


    public DepoTurret(CRServo turret, AnalogInput angularEncoder) {
        this.turret = turret;
        this.angularEncoder = angularEncoder;
        this.angle = new axonEncoder(angularEncoder);
        this.servoController = new AnglePID(new AnglePID.Coefficients(Kp, Ki, Kd),
                () -> this.updateAngle()- this.updateTargetAngle(),

                factor -> this.turret.setPower(M.clamp(factor, -.5, .5)));

    }

    public void update() {
        this.target = updateTargetAngle();
        this.servoController.update();

    }
    public double updateTargetAngle(){
        return target;
    }

    public double updateAngle(){
        currentAngle = angle.update();
        if(Math.abs(currentAngle - lastAngle) > 180){
            count += Math.signum(lastAngle - currentAngle);
        }
        lastAngle = currentAngle;
        return count * 360 + currentAngle;
    }

}
