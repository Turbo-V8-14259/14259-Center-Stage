package org.firstinspires.ftc.teamcode.vision.tests;


import static org.firstinspires.ftc.teamcode.vision.Red.Location.MIDDLE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Red;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="VisionOpRed")
public class RedOp extends LinearOpMode {
    private Red.Location location = MIDDLE;
    private Red redPropProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        redPropProcessor = new Red(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor);

        while (!isStarted()) {
            location = redPropProcessor.getLocation();
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


