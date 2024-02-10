package org.firstinspires.ftc.teamcode.vision.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
/**
 * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
 */

/**
 * {@link #visionPortal} is the variable to store our instance of the vision portal.
 */

@TeleOp(name = "AprilTag Detection Software")
@Disabled
//@Disabled
public class AprilTagTest extends LinearOpMode{

//AprilTag Specific Stuff
    private AprilTagProcessor aprilTag;

    private final AprilTagPipeline ac = new AprilTagPipeline();
    private VisionPortal visionPortal;
    public AprilTagDetection detection;
    int toi = 5;
//Motor Stuff
    private DcMotorBetter sampleMotor; //no motors yet here for reasons
    //Dashboard
    //soon


    public void runOpMode() {

        initAll();


        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();
                telemetry.update();

                detection = AprilTagPipeline.getSpecificTagData(aprilTag,toi);
                if(detection != null){

                }
                else{

                }

                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }

    private void initAprilTag() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTag = ac.initAprilTag();
        visionPortal = ac.initVision(webcamName);

//        VisionPortal.Builder builder = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .setCameraResolution(new Size(800, 600))
//                .enableLiveView(true)
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .setAutoStopLiveView(false)
//                .addProcessor(aprilTag);
//        VisionPortal visionPortal = builder.build();

        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    private void initAll(){
        initAprilTag();
    }

    private void initMotors(){
        //soon
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        if(AprilTagPipeline.detectSpecificTag(aprilTag,toi)){
            if(detection != null) {
                telemetry.addData("Distance from AprilTag (Inches): ", detection.ftcPose.y);
            }
        }
        else{
            telemetry.addLine("no lmao");
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        //telemetry.addLine(String.format(" ID %6.1f Detected, X Y Z %6.1f %6.1f %6.1f  (Inch)", detection.id, detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
    }

}