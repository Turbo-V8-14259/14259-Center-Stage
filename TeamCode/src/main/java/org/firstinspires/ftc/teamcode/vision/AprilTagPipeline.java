package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
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

    public VisionPortal visionPortal;
    //public WebcamName webcamName;

    public static AprilTagProcessor initAprilTag() { //add second webcam if we use 2

        return new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
//        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public static VisionPortal initVision(WebcamName webcamName, AprilTagProcessor aprilTag){
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new Size(800, 600))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .addProcessor(aprilTag);

        return builder.build();
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
            }
        }
        return tagDetect;
    }
    public static AprilTagDetection getSpecificTagData(AprilTagProcessor aprilTag, int tag){
        AprilTagDetection specificTag = null;
        if(detectSpecificTag(aprilTag,tag)){
            ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
            for(int i = 0; i<detections.size(); i++){
                if(detections.get(i).id == tag){
                    specificTag = detections.get(i);
                }
            }
            return specificTag;
        }
        else{
            return null;
        }
    }
    public static AprilTagDetection[] getSpecificTagDatas(AprilTagProcessor aprilTag, int[] tag){
        AprilTagDetection[] specificTags = new AprilTagDetection[tag.length];
        for(int j = 0; j<99; j++){
            if(detectSpecificTag(aprilTag,j)){
                ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
                for(int i = 0; i<detections.size(); i++){
                    if(detections.get(i).id == j){
                        specificTags[j] = detections.get(i);
                    }
                    else{
                        specificTags[j] = null;
                    }
                }
            }
        }
        return specificTags;
    }
    public int amountDetected(AprilTagProcessor aprilTag){
        return aprilTag.getDetections().size();
    }
    //x location : 58
    //apriltag 1 & 6: y = +- 41
    //apriltag 2 & 5: y = +- 35
    //apriltag 3 & 4: y = +-  29
    public static double[] getCoordsRelToAprilTag(AprilTagDetection detection){
        double x;
        double y = 0;
        double tileLength = 23.5;
        if(detection != null){
            x = 59 - detection.ftcPose.y;
            if(CameraPipeline.isBlue()){
                switch (detection.metadata.id) {
                    case 1:
                        y = tileLength + (double) 3 / 4 * tileLength;
                        break;
                    case 2:
                        y = tileLength + (double) 1 / 2 * tileLength;
                        break;
                    case 3:
                        y = tileLength + (double) 1 / 4 * tileLength;
                        break;
                }
            }
            else if(CameraPipeline.isRed()){
                switch (detection.metadata.id){
                    case 4:
                        y = tileLength + (double) 1 /4 * tileLength;
                        break;
                    case 5:
                        y = tileLength + (double) 1 /2 * tileLength;
                        break;
                    case 6:
                        y = tileLength + (double) 3 /4 * tileLength;
                        break;
                }
                y = -y;
            }
            y += -detection.ftcPose.x;
            return new double[]{x,y};
        }
        else{
            return null;
        }
    }
}

