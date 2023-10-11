package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.AnglePID;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;
import org.firstinspires.ftc.teamcode.usefuls.Sensors.axonEncoder;
@TeleOp
public class servoPIDTest extends LinearOpMode {
    private axonEncoder angle;
    private AnalogInput angularEncoder;
    private CRServo a;
    private AnglePID servoController;
    private double Kp = 0, Ki = 0, Kd = 0;

    private double targetServoAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        angularEncoder = hardwareMap.get(AnalogInput.class, "input");
        angle = new axonEncoder(angularEncoder);
        a = hardwareMap.get(CRServo.class, "axon");
        this.servoController = new AnglePID(new AnglePID.Coefficients(Kp, Ki, Kd),
                () -> this.targetServoAngle - this.angle.update(),
                factor -> {
                    this.a.setPower(M.clamp(-factor, .5, -.5));

                });
        
        waitForStart();

        while(opModeIsActive()){

            telemetryUpdate();
        }
    }
    public void telemetryUpdate() {
        telemetry.addData("Angle in Degrees; ", angle.update());
        telemetry.update();
    }

}
