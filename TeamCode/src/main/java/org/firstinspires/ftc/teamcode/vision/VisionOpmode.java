package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.leftPer;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.midPer;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.rightPer;
import static org.firstinspires.ftc.teamcode.vision.CameraPipeline.color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class VisionOpmode extends LinearOpMode
{
    //it would probably be simpler to port this from the camerapipeline but shhh this was made a while ago im only updating it now.
    OpenCvWebcam webcam;
    public String ObjectDirection;
    public int randomization = 99;
    private VisionPortal visionPortal;
    public double thresh = 10;

    @Override
    public void runOpMode()
    {
        telemetry.addLine("Loading Pipeline...");
        telemetry.update();
        initPipeline();

        while(ObjectDirection == null){
            telemetry.addLine("Identifying Location...");
            sleep(2500);

            if(leftPer > thresh || rightPer > thresh || midPer > thresh){
                if(leftPer > rightPer && leftPer > midPer){ //mid
                    if(color.equals("BLUE")){
                        ObjectDirection = "LEFT";
                    }
                    else if(color.equals("RED")){
                        ObjectDirection = "MIDDLE";
                    }
                }
                else if(rightPer > leftPer && rightPer > midPer){ //right
                    if(color.equals("BLUE")){
                        ObjectDirection = "MIDDLE";
                    }
                    else if(color.equals("RED")){
                        ObjectDirection = "RIGHT";
                    }
                }
            }

            else{
                if(color.equals("BLUE")){
                    ObjectDirection = "RIGHT";
                }
                else if(color.equals("RED")){
                    ObjectDirection = "LEFT";
                }
            }

            switch (ObjectDirection) {
                case "LEFT":
                    randomization = 0;
                    break;
                case "RIGHT":
                    randomization = 2;
                    break;
                case "MIDDLE":
                    randomization = 1;
                    break;
            }
            telemetry.update();
        }

        telemetry.addData("Location:", ObjectDirection);
        telemetry.update();

        sleep(50);

        webcam.stopRecordingPipeline();
        webcam.closeCameraDevice();

        telemetry.update();
//        telemetry.addLine("Loading AprilTagDetections...");
//        telemetry.update();
//
//        initAprilTag();
//        visionPortal.setProcessorEnabled(aprilTag, true);

        telemetry.addLine("Ready to Start");
        telemetry.addData("Location:", ObjectDirection);
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            // AprilTag (wont need until later)
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
    }
    private void initPipeline(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        CameraPipeline s = new CameraPipeline(telemetry);
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

        sleep(1000);

    }
    public int randomization(){
        int random = 0;
        if(leftPer > thresh || rightPer > thresh || midPer > thresh){
            if(leftPer > rightPer && leftPer > midPer){ //left
                random = 1;
            }
            else if(rightPer > leftPer && rightPer > midPer){ //right
                random = 2;
            }
        }
        else{
            random = 0;
        }
        return random;
    }

    //            if(leftPer > thresh || rightPer > thresh || midPer > thresh){
//                if(leftPer > rightPer && leftPer > midPer){ //left
//                    ObjectDirection = "LEFT";
//                }
//                else if(rightPer > leftPer && rightPer > midPer){ //right
//                    ObjectDirection = "RIGHT";
//                }
//                else if(midPer > rightPer && midPer > leftPer){ //mid
//                    ObjectDirection = "MIDDLE";
//                }
//            }
//            else{
//                thresh--;
//                telemetry.addLine("Retrying...");
//                telemetry.addData("Value L:", leftPer);
//                telemetry.addData("Location R:", rightPer);
//                telemetry.addData("Location M:", midPer);
//            }
}
