package org.firstinspires.ftc.teamcode.vision.tests;

import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.color;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.leftPer;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.rightPer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.CameraPipeline;
import org.firstinspires.ftc.teamcode.vision.PatternDetectionTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp

public class OpenCVTestRed extends LinearOpMode
{
    OpenCvWebcam webcam;
    String ObjectDirection;
    int thresh = 15;
    int randomization = 1;

    @Override
    public void runOpMode()
    {
        webcam = CameraPipeline.initPipeline(hardwareMap, telemetry);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        CameraPipeline.setColor("RED");

        while(opModeInInit() && !isStopRequested()){
            ObjectDirection = CameraPipeline.randomization(thresh);
            randomization = CameraPipeline.PosToNum(ObjectDirection);

            telemetry.addLine("Ready to Start");
            telemetry.addData("Location:", ObjectDirection);
            telemetry.addData("Color:", color);
            telemetry.addData("Right:", rightPer);
            telemetry.addData("Left:", leftPer);

            telemetry.update();
        }

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            telemetry.addData("Color", PatternDetectionTest.op);

            telemetry.update();
            sleep(100);
            //continue the auton path here
        }
    }
}