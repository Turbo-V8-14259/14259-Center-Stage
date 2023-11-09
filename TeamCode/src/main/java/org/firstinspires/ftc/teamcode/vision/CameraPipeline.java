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
    Telemetry telemetry;

    static final Rect LEFT_ROI = new Rect(
            new Point(10, 0),
            new Point(100, 200));
    static final Rect MID_ROI = new Rect(
            new Point(110, 0),
            new Point(200, 200));
    static final Rect RIGHT_ROI = new Rect(
            new Point(210, 0),
            new Point(300, 200));
    static double PERCENT_COLOR_THRESHOLD = 0.3;
    String ObjectDirection;
    Mat mat = new Mat();

    public CameraPipeline(Telemetry t, String s) {
        telemetry = t;
        ObjectDirection = s;
    }

    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); //Uses HSV Colors

//        Scalar lowHSVRed = new Scalar(0,100,20); // lower bound HSV for red 0 100 20
//        Scalar highHSVRed = new Scalar(10, 255, 255); // higher bound HSV for red 10 255 255

        Scalar lowHSVBlue = new Scalar(110, 100, 20); // lower bound HSV for blue 110 100 20
        Scalar highHSVBlue = new Scalar(130, 255, 255); // higher bound HSV for blue 130 255 255

        Mat thresh = new Mat();


        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat mid = mat.submat(MID_ROI);



        //Core.inRange(mat, lowHSVRed, highHSVRed, thresh);
        Core.inRange(mat, lowHSVBlue, highHSVBlue, thresh);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();
        mid.release();

        telemetry.addData("Left raw value", Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value",  Core.sumElems(right).val[0]);
        telemetry.addData("Mid raw value", Core.sumElems(mid).val[0]);

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Mid percentage", Math.round(midValue * 100) + "%");



        boolean objLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean objRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean objMid = midValue > PERCENT_COLOR_THRESHOLD;

        if(objLeft && objRight){
            if(leftValue > rightValue){
                objLeft = true;
                objRight = false;
            }
            else{
                objLeft = false;
                objRight = true;
            }
        }
        if(objMid && objRight){
            if(midValue > rightValue){
                objMid = true;
                objRight = false;
            }
            else{
                objMid = false;
                objRight = true;
            }
        }
        if(objLeft && objMid){
            if(leftValue > midValue){
                objLeft = true;
                objMid = false;
            }
            else{
                objLeft = false;
                objMid = true;
            }
        }
        if(objLeft){
            ObjectDirection = "LEFT";
            Imgproc.rectangle(
                    input, //mat
                    LEFT_ROI,
                    new Scalar(0, 255, 0), 4);
        }
        else if(objRight){
            ObjectDirection = "RIGHT";
            Imgproc.rectangle(
                    input, //mat
                    RIGHT_ROI,
                    new Scalar(0, 255, 0), 4);
        }
        else if(objMid){
            ObjectDirection = "MIDDLE";
            Imgproc.rectangle(
                    input, //mat
                    MID_ROI,
                    new Scalar(0, 255, 0), 4);
        }
        else{
            ObjectDirection = "NONE";
        }

        Mat screen = new Mat();


//        Imgproc.rectangle(
//                mat,
//                LEFT_ROI,
//                new Scalar(0, 255, 0), 4);
//        Imgproc.rectangle(
//                mat,
//                RIGHT_ROI,
//                new Scalar(0, 255, 0), 4);
//        Imgproc.rectangle(
//                mat,
//                MID_ROI,
//                new Scalar(0, 255, 0), 4);

        telemetry.addData("Location: ", ObjectDirection);



        return input;

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

    }
}
