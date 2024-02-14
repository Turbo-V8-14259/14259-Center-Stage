package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Objects;

public class CameraPipeline extends OpenCvPipeline
{
    public static String color = "RED"; //change this every match accordingly
    //or find some way to change this easier
    //if you want me to make a completely new pipeline for each side just say so

    public static double perThreshold = 15;
    Telemetry telemetry;
    static Rect LEFT_ROI = null;
    static Rect RIGHT_ROI = null;
    public static String ObjectDirection;
    Mat mat = new Mat();

    public CameraPipeline(Telemetry t){
        telemetry = t;
    }

    public static double leftPer;
    public static double rightPer;
    public static double midPer;

    boolean objLeft;
    boolean objRight;

    public Mat processFrame(Mat input)
    {

        Size s = input.size();
        double height = s.height;
        double width = s.width;

        if(Objects.equals(color, "BLUE")){ //remind me to adjust
            LEFT_ROI = new Rect(
                    new Point(1.0 / 8 * width, 3.0/8 * height),
                    new Point(3.0/8 * width, 5.0/8 * height));

            RIGHT_ROI = new Rect(
                    new Point(5.0 * width/8, 3.0/8 * height),
                    new Point(7.0/8 * width, 5.0/8 * height));
        }
        else if (Objects.equals(color, "RED")) {
            LEFT_ROI = new Rect(
                    new Point(1.0/4 * width, 3.0/8 * height),
                    new Point(1.0 * width/2, 5.0/8 * height));

            RIGHT_ROI = new Rect(
                    new Point(5.0 * width/8, 3.0/8 * height),
                    new Point(7.0/8 * width, 1.0/2 * height));
        }


        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); //Uses HSV Colors

        Scalar lowHSVRed = new Scalar(0,150,20); // lower bound HSV for red 0 100 20
        Scalar highHSVRed = new Scalar(25, 255, 255); // higher bound HSV for red 10 255 255

        Scalar lowHSVBlue = new Scalar(100, 100, 20); // lower bound HSV for blue 110 100 20
        Scalar highHSVBlue = new Scalar(130, 255, 255); // higher bound HSV for blue 130 255 255

        Mat thresh = new Mat();

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);


        if(Objects.equals(color, "RED")){
            Core.inRange(mat, lowHSVRed, highHSVRed, thresh);
        }
        else if (Objects.equals(color, "BLUE")) {
            Core.inRange(mat, lowHSVBlue, highHSVBlue, thresh);
        }

        Mat leftT = thresh.submat(LEFT_ROI);
        Mat rightT = thresh.submat(RIGHT_ROI);

        double leftValThr = Core.sumElems(leftT).val[0] / LEFT_ROI.area() / 255;
        double rightValThr = Core.sumElems(rightT).val[0] / RIGHT_ROI.area() / 255;

        leftPer = Math.round(leftValThr * 100);
        rightPer = Math.round(rightValThr * 100);
        midPer = 0;

        //double midValue = Core.sumElems(mid).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        leftT.release();
        rightT.release();

        //mid.release();

        objLeft = leftPer > perThreshold;
        objRight = rightPer > perThreshold;

        if(objLeft && objRight){
            if(leftPer > rightPer){
                objLeft = true;
                objRight = false;
            }
            else{
                objRight = true;
                objLeft = false;
            }
        }

        if(objLeft){
            if (Objects.equals(color, "RED")) {
                ObjectDirection = "MIDDLE";
            }
            if (Objects.equals(color, "BLUE")) {
                ObjectDirection = "MIDDLE";
            }
        }
        else if(objRight){
            if (color == "BLUE") {
                ObjectDirection = "RIGHT";
            }
            if (color == "RED") {
                ObjectDirection = "RIGHT";
            }
        }
        else{
            if(Objects.equals(color, "BLUE")){
                ObjectDirection = "LEFT";
            }else if(Objects.equals(color, "RED")){
                ObjectDirection = "LEFT";
            }
            Imgproc.rectangle(
                    thresh,
                    LEFT_ROI,
                    new Scalar(255, 255, 255), 4);
            Imgproc.rectangle(
                    thresh,
                    RIGHT_ROI,
                    new Scalar(255, 255, 255), 4);
        }



//        Imgproc.rectangle(
//                thresh,
//                MID_ROI,
//                new Scalar(255, 255, 255), 4);

       // telemetry.addData("Location: ", ObjectDirection);
        //telemetry.update();



        return input; //input

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

    }
    public static void setColor(String color){
        CameraPipeline.color = color;
    }
    public static void initPipeline(OpenCvWebcam webcam, HardwareMap hardwareMap, Telemetry telemetry) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        CameraPipeline s = new CameraPipeline(telemetry);
        webcam.setPipeline(s);

        webcam.setMillisecondsPermissionTimeout(5000); 
        OpenCvWebcam finalWebcam = webcam;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
}