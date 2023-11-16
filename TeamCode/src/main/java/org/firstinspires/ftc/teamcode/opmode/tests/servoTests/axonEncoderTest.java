package org.firstinspires.ftc.teamcode.opmode.tests.servoTests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Sensors.axonEncoder;
@TeleOp
@Disabled

public class axonEncoderTest extends LinearOpMode {
    private axonEncoder test;
    private AnalogInput encoder;
    private CRServo a;


    int count = 0;
    double currentAngle;
    double lastAngle;

    double actualAngle;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initializeComponents();
        waitForStart();

        while(opModeIsActive()){

            currentAngle = test.update();
            if(Math.abs(currentAngle - lastAngle) > 180){
                count += Math.signum(lastAngle - currentAngle);
            }


            a.setPower(0);


            actualAngle = count * 360 + currentAngle;
            telemetry.addData("angle", actualAngle);

            telemetry.addData("raw", test.update());
            telemetry.addData("count: ", count);
            telemetry.update();


            lastAngle = currentAngle;
        }
    }

    private void initializeComponents() {
        encoder = hardwareMap.get(AnalogInput.class, "input");
        test = new axonEncoder(encoder);
        a = hardwareMap.get(CRServo.class, "axon");
    }

}
