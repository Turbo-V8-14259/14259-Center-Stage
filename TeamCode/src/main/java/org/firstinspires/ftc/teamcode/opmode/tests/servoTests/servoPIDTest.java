package org.firstinspires.ftc.teamcode.opmode.tests.servoTests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.AnglePID;
import org.firstinspires.ftc.teamcode.hardware.Sensors.axonEncoder;
@TeleOp
@Config
@Disabled
public class servoPIDTest extends LinearOpMode {
    private axonEncoder angle;
    private AnalogInput angularEncoder;
    private CRServo a;
    private AnglePID servoController;
    public static double Kp = 0.013, Ki = 0.003, Kd = 0.0005;
    public static double targetServoAngle;
    int count = 0;
    double currentAngle;
    double lastAngle;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        angularEncoder = hardwareMap.get(AnalogInput.class, "input");
        angle = new axonEncoder(angularEncoder);


        a = hardwareMap.get(CRServo.class, "axon");

        this.servoController = new AnglePID(new AnglePID.Coefficients(Kp, Ki, Kd),
                //() -> this.updateAngle()- this.updateTargetAngle(),
                () -> this.updateAngle()- this.updateTargetAngle(),

                factor -> this.a.setPower(M.clamp(factor, -1, 1)));

        waitForStart();

        while(opModeIsActive()){
            servoController.update();
            telemetryUpdate();
        }
    }
    public void telemetryUpdate() {
        telemetry.addData("Angle in Degrees WRAPPED", updateAngle());
        telemetry.addData("The target angle in Degrees is: ", targetServoAngle);
        telemetry.update();
    }
    public double updateAngle(){
        currentAngle = angle.update();
        if(Math.abs(currentAngle - lastAngle) > 180){
            count += Math.signum(lastAngle - currentAngle);
        }
        lastAngle = currentAngle;
        return count * 360 + currentAngle;
    }
    public double updateTargetAngle(){
        return targetServoAngle;
    }

}
