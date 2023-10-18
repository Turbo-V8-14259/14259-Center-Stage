package org.firstinspires.ftc.teamcode.opmode.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.AnglePID;
import org.firstinspires.ftc.teamcode.hardware.Sensors.axonEncoder;
@TeleOp
@Config
public class servoPIDTest extends LinearOpMode {
    private axonEncoder angle;
    private AnalogInput angularEncoder;
    private CRServo a;
    private AnglePID servoController;
    private double Kp = 0.5, Ki = 0, Kd = 0;
    private double targetServoAngle = 0;
    private double angleInDeg;

    @Override
    public void runOpMode() throws InterruptedException {

        angularEncoder = hardwareMap.get(AnalogInput.class, "input");
        angle = new axonEncoder(angularEncoder);


        a = hardwareMap.get(CRServo.class, "axon");

        this.servoController = new AnglePID(new AnglePID.Coefficients(Kp, Ki, Kd),
                () -> this.angle.update()- this.targetServoAngle,
                factor -> this.a.setPower(M.clamp(factor, -.1, .1)));


        waitForStart();

        while(opModeIsActive()){
            targetServoAngle = 90;
            servoController.update();
            telemetryUpdate();
        }
    }
    public void telemetryUpdate() {
        telemetry.addData("Angle in Degrees; ", angle.update());
        telemetry.addData("The target angle in Degrees is: ", targetServoAngle);
        telemetry.update();
    }

}
