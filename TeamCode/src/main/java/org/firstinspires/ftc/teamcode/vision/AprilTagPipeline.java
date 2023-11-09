package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagPipeline{

    public boolean ifDetectingApriltag(AprilTagProcessor aprilTag){
        return !aprilTag.getDetections().isEmpty();
    }
    public AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(false)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build();;
    public VisionPortal visionPortal;
    //public WebcamName webcamName;

    public AprilTagProcessor initAprilTag() { //add second webcam if we use 2
        return this.aprilTag;
//        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public VisionPortal initVision(WebcamName webcamName){
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new Size(800, 600))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .addProcessor(aprilTag);

        VisionPortal visionPortal1 = builder.build();

        return visionPortal1;
    }

    private void disableAprilTag(AprilTagProcessor aprilTag, VisionPortal visionPortal){
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    //this function works
    public static boolean detectSpecificTag(AprilTagProcessor aprilTag, int tagName){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean tagDetect = false;
        if(detections == null){
            return false;
        }
        for(AprilTagDetection detection : detections){
            if(detection.metadata != null){
                if(detection.id == tagName){
                    tagDetect = true;
                    break;
                }
                else{
                    tagDetect = false;
                }
            }
            else{
                tagDetect = false;
            }
        }
        return tagDetect;
    }

    //TO DO:


    //dont really need this anymore
    //still could use this i guess
    public double[] AprilTagPosition(AprilTagProcessor aprilTag, int tag){
        ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
        double[] coords = new double[2];
        for(AprilTagDetection detection : detections){
            if(detection.metadata != null){
                if(detectSpecificTag(aprilTag,tag)){
                    coords[0] = detection.ftcPose.x;
                    coords[1] = detection.ftcPose.y;
                    coords[2] = detection.ftcPose.z;
                }
            }
            else{
                return null;
            }
        }
        return coords;
    }

    public static AprilTagDetection getSpecificTagData(AprilTagProcessor aprilTag, int tag){
        AprilTagDetection specificTag = null;
        if(detectSpecificTag(aprilTag,tag)){
            ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
            for(int i = 0; i<detections.size(); i++){
                if(detections.get(i).id == tag){
                    specificTag = detections.get(i);
                }
                else{

                }
            }
            return specificTag;
        }
        else{
            return null;
        }
    }

    public int amountDetected(AprilTagProcessor aprilTag){
        return aprilTag.getDetections().size();
    }

//    public ArrayList<AprilTagDetection> getDetections() {
//        return this.aprilTag.getDetections();
//    }
}

