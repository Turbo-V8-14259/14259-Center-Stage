package org.firstinspires.ftc.teamcode.vision.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.CameraPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.*;
@TeleOp
@Disabled
public class AutoVisionTester extends LinearOpMode
{
    boolean Blue = false;
    boolean Red = true;
    OpenCvWebcam webcam;
    public static String ObjectDirection;


    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private final AprilTagPipeline ac = new AprilTagPipeline();

    public AprilTagDetection detection;

    @Override
    public void runOpMode()
    {
        initAll();
        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();
        while (opModeIsActive())
        {

            final String detectionDR;
            detectionDR = ObjectDirection;
            sleep(5000);
            webcam.stopRecordingPipeline();

            visionPortal.setProcessorEnabled(aprilTag, true);


            if(detectionDR == "LEFT"){
                if(Red){
                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,4);
                }
                else if(Blue){
                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,1);
                }
            }
            else if(detectionDR == "MIDDLE"){
                if(Red){
                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,5);
                }
                else if(Blue){
                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,2);
                }
            }
            else if(detectionDR == "RIGHT"){
                if(Red){
                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,6);
                }
                else if(Blue){
                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,3);
                }
            }
            if(detection != null){

            }
            else{

            }
            telemetry.addData("Direction", detectionDR);
            telemetry.update();
        }
    }














    private void initAprilTag() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTag = ac.initAprilTag();
        visionPortal = ac.initVision(webcamName);

        visionPortal.setProcessorEnabled(aprilTag, false);
    }
    private void initPipeline(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        CameraPipeline s = new CameraPipeline(telemetry, ObjectDirection);
        webcam.setPipeline(s);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
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
    }
    private void initAll(){
        initAprilTag();
        initPipeline();
    }

}
