package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.hardware.Sensors.gfAnalogTracker;

public class gfTracketTest extends LinearOpMode {
    private AnalogInput tracker;
    private gfAnalogTracker actualTracker;
    public void runOpMode() throws InterruptedException{
        tracker = (AnalogInput) hardwareMap.get("tracker_r");
        actualTracker = new gfAnalogTracker(tracker);
        actualTracker.reset();
        waitForStart();

        while(opModeIsActive()){
            actualTracker.update();
            updateTelemetry();
        }
    }
    public void updateTelemetry(){
        telemetry.addData("pos: ", actualTracker.getPos());
        telemetry.addData("pos: ", actualTracker.getRAW());

        telemetry.update();
    }
}
