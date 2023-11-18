package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraPipeline extends OpenCvPipeline
{
//    public static String color = "BLUE";
    public static String color = "BLUE";


    Telemetry telemetry;

    static final Rect LEFT_ROI = new Rect(
            new Point(0, 100),
            new Point(150, 225));
    //    static final Rect MID_ROI = new Rect(
//            new Point(110, 150),
//            new Point(200, 225));
    static final Rect RIGHT_ROI = new Rect(
            new Point(150, 100),
            new Point(300, 225));
    static double PERCENT_COLOR_THRESHOLD = 0.22;
    String ObjectDirection;
    Mat mat = new Mat();

    public CameraPipeline(Telemetry t, String s) {
        telemetry = t;
        ObjectDirection = s;
    }

    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); //Uses HSV Colors

        Scalar lowHSVRed = new Scalar(0,100,20); // lower bound HSV for red 0 100 20
        Scalar highHSVRed = new Scalar(10, 255, 255); // higher bound HSV for red 10 255 255

        Scalar lowHSVBlue = new Scalar(110, 50, 20); // lower bound HSV for blue 110 100 20
        Scalar highHSVBlue = new Scalar(130, 255, 255); // higher bound HSV for blue 130 255 255

        Mat thresh = new Mat();

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        //Mat mid = mat.submat(MID_ROI);

        if(color == "RED"){
            Core.inRange(mat, lowHSVRed, highHSVRed, thresh);
        }
        else if (color == "BLUE") {
            Core.inRange(mat, lowHSVBlue, highHSVBlue, thresh);
        }



        Mat leftT = thresh.submat(LEFT_ROI);
        Mat rightT = thresh.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        double leftValThr = Core.sumElems(leftT).val[0] / LEFT_ROI.area();
        double rightValThr = Core.sumElems(rightT).val[0] / LEFT_ROI.area();



        double leftvalueRaw = Core.sumElems(left).val[0];
        double rightvalueRaw = Core.sumElems(right).val[0];

        double leftPer = Math.round(leftValue * 100) /*modifier*/;
        double rightPer = Math.round(rightValue * 100);
        //double midValue = Core.sumElems(mid).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();
        //mid.release();

        telemetry.addData("Left value Thr", leftValThr);
        telemetry.addData("Right value Thr",  rightValThr);

        telemetry.addData("Left value", leftValue);
        telemetry.addData("Right value",  rightValue);



        boolean objLeft = leftValThr > 5;
        boolean objRight = rightValThr > 5;

//        if(Red){
//            if(leftPer - 20 >= 3){
//                objLeft = true;
//            }
//            if(rightValue == 26){
//                objRight = true;
//            }
//        }
        //boolean objMid = midValue > PERCENT_COLOR_THRESHOLD;

//        if(objLeft && objRight){
//            if(leftValue > rightValue){
//                objLeft = true;
//                objRight = false;
//            }
//            else{
//                objLeft = false;
//                objRight = true;
//            }
//        }
//        if(objMid && objRight){
//            if(midValue > rightValue){
//                objMid = true;
//                objRight = false;
//            }
//            else{
//                objMid = false;
//                objRight = true;
//            }
//        }
//        if(objLeft && objMid){
//            if(leftValue > midValue){
//                objLeft = true;
//                objMid = false;
//            }
//            else{
//                objLeft = false;
//                objMid = true;
//            }
//        }



//        if(objLeft && objRight){
//            if(leftValue > rightValue){
//                objLeft = true;
//                objRight = false;
//            }
//            else{
//                objRight = true;
//                objLeft = false;
//            }
//        }



        if(objLeft){
            if (color == "BLUE") {
                ObjectDirection = "MIDDLE";
            }
            if (color == "RED") {
                ObjectDirection = "LEFT";
            }

            Imgproc.rectangle(
                    input, //mat
                    LEFT_ROI,
                    new Scalar(255, 255, 255), 4);
        }
        else if(objRight){
            if (color == "BLUE") {
                ObjectDirection = "RIGHT";
            }
            if (color == "RED") {
                ObjectDirection = "MIDDLE";
            }
            Imgproc.rectangle(
                    input, //mat
                    RIGHT_ROI,
                    new Scalar(255, 255, 255), 4);
        }
//        else if(objMid){
//            Imgproc.rectangle(
//                    thresh, //mat
//                    MID_ROI,
//                    new Scalar(255, 255, 255), 4);
//        }
        else{
            if(color == "BLUE"){
                ObjectDirection = "LEFT";
            }else if(color =="RED"){
                ObjectDirection = "RIGHT";
            }
        }


//        Imgproc.rectangle(
//                thresh,
//                LEFT_ROI,
//                new Scalar(255, 255, 255), 4);
//        Imgproc.rectangle(
//                thresh,
//                RIGHT_ROI,
//                new Scalar(255, 255, 255), 4);
//        Imgproc.rectangle(
//                thresh,
//                MID_ROI,
//                new Scalar(255, 255, 255), 4);

        telemetry.addData("Location: ", ObjectDirection);
        telemetry.update();



        return thresh; //input

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

    }
}