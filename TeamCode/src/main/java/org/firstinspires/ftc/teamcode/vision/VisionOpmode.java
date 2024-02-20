package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.leftPer;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.rightPer;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class VisionOpmode extends LinearOpMode
{
    //Regular Vision
    OpenCvWebcam webcam;
    public String ObjectDirection;
    public int randomization = 99;
    public double thresh = CameraPipeline.perThreshold;

    //AprilTag
    private final AprilTagPipeline ac = new AprilTagPipeline();
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection detection;


    @Override
    public void runOpMode()
    {
        telemetry.addLine("Loading Pipeline...");
        CameraPipeline.setColor("RED");
        telemetry.update();
        webcam = CameraPipeline.initPipeline(hardwareMap, telemetry);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addLine("Loading AprilTagDetections...");

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTag = ac.initAprilTag();
        visionPortal = AprilTagPipeline.initVision(webcamName);
        telemetry.update();

        while(opModeInInit()){
            ObjectDirection = CameraPipeline.randomization(thresh);
            randomization = CameraPipeline.PosToNum(ObjectDirection);

            telemetry.addLine("Ready to Start");
            telemetry.addData("Location:", ObjectDirection);
            telemetry.addData("Color:", color);
            telemetry.addData("Right:", rightPer);
            telemetry.addData("Left:", leftPer);

            telemetry.update();

            if(isStopRequested()){
                webcam.stopRecordingPipeline();
                webcam.closeCameraDevice();
                return;
            }
        }

        waitForStart();
        webcam.stopRecordingPipeline();
        webcam.closeCameraDevice();
        visionPortal.setProcessorEnabled(aprilTag, true);

        while (opModeIsActive())
        {
            // AprilTag
//            if(Objects.equals(ObjectDirection, "LEFT")){
//                if(Red){
//                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,4);
//                }
//                else if(Blue){
//                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,1);
//                }
//            }
//            else if(Objects.equals(ObjectDirection, "MIDDLE")){
//                if(Red){
//                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,5);
//                }
//                else if(Blue){
//                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,2);
//                }
//            }
//            else if(Objects.equals(ObjectDirection, "RIGHT")){
//                if(Red){
//                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,6);
//                }
//                else if(Blue){
//                    detection = AprilTagPipeline.getSpecificTagData(aprilTag,3);
//                }
//            }
//
//            if(detection != null){
//
//            }
//            else{
//
//            }
            telemetry.addData("Location", ObjectDirection);
            telemetry.update();
        }

        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.close();
    }
}
