package org.firstinspires.ftc.teamcode.vision.tests;


import static org.firstinspires.ftc.teamcode.vision.Blue.Location.MIDDLE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Blue;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="VisionOpBlue")
public class BlueOp extends LinearOpMode {
    private Blue.Location location = MIDDLE;
    private Blue bluePropProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        bluePropProcessor = new Blue(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), bluePropProcessor);

        while (!isStarted()) {
            location = bluePropProcessor.getLocation();
            telemetry.update();
        }
        waitForStart();
        /*while(opModeIsActive()){

        }*/
        switch (location) {
            case LEFT:
                break;
            case MIDDLE:
                break;
            case RIGHT:
                break;
        }
    }
}