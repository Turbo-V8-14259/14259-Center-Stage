package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueContour extends OpenCvPipeline
{
    Telemetry telemetry;
    public String position;

    public enum Position{
        LEFT,
        RIGHT,
        MIDDLE
    }

    public static Position p = Position.LEFT;

    public BlueContour(Telemetry t){
        telemetry = t;
    }

    public Mat processFrame(Mat input)
    {
        Size s = input.size();

        double height = s.height;
        double width = s.width;

        int column_width_1 = (int) (width/3);
        int column_width_2 = (int) (width/3) * 2;

        telemetry.addLine(String.valueOf(height));
        telemetry.addLine(String.valueOf(width));

        telemetry.update();

        //temp
        Scalar lower_blue = new Scalar(120, 100, 70);
        Scalar upper_blue = new Scalar(140, 225, 225);

        Mat mask = new Mat();
        Mat mat = new Mat();
        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, lower_blue, upper_blue, mask);
        //temp

        Scalar line_color = new Scalar(255,255,255);
        int line_thickness = 2;

        //draw line
        Imgproc.line(mask, new Point(column_width_1, 0), new Point(column_width_1, height), line_color, line_thickness);
        Imgproc.line(mask, new Point(width - column_width_2, 0), new Point(width - column_width_2, height), line_color, line_thickness);

        //define submat
        Mat roi_left = input.submat(new Rect(new Point(0,0), new Point(column_width_1, height)));
        Mat roi_middle = input.submat(new Rect(new Point(column_width_1, 0), new Point(column_width_2, height)));
        Mat roi_right = input.submat(new Rect(new Point(column_width_2,0), new Point(width, height)));

        //draw test rect
        Imgproc.rectangle(mask, new Rect(new Point(0,0), new Point(column_width_1, height)), line_color, line_thickness);
        Imgproc.rectangle(mask, new Rect(new Point(column_width_1, 0), new Point(column_width_2, height)), line_color, line_thickness);
        Imgproc.rectangle(mask, new Rect(new Point(column_width_2, 0), new Point(width, height)), line_color, line_thickness);

        //get area
        Mat roi_hsv_left = new Mat();
        Mat roi_hsv_middle = new Mat();
        Mat roi_hsv_right = new Mat();

        Imgproc.cvtColor(roi_left, roi_hsv_left, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(roi_middle, roi_hsv_middle, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(roi_right, roi_hsv_right, Imgproc.COLOR_BGR2HSV);

        double area_left = find_largest_blue_area(roi_hsv_left);
        double area_middle = find_largest_blue_area(roi_hsv_middle);
        double area_right = find_largest_blue_area(roi_hsv_right);

        //memory leak is bad
        roi_hsv_left.release();
        roi_left.release();

        roi_middle.release();
        roi_hsv_middle.release();

        roi_right.release();
        roi_hsv_right.release();

        mat.release();

        //logic
        if(area_left > area_middle && area_left > area_right){
            position = "LEFT";
            p = Position.LEFT;
            telemetry.addLine("LEFT");
        }
        else if(area_middle > area_left && area_middle > area_right){
            position = "MIDDLE";
            p = Position.MIDDLE;
            telemetry.addLine("MIDDLE");
        }
        else if(area_right > area_left && area_right > area_middle){
            position = "RIGHT";
            p = Position.RIGHT;
            telemetry.addLine("RIGHT");
        }
        else{
            telemetry.addLine("LEFT");
        }

        return mask; //input
        //basically this is the "mat (image)" that would be output

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

    }

    public double find_largest_blue_area(Mat roi_hsv){
        Scalar lower_blue = new Scalar(120, 100, 70); //0 100 70
        Scalar upper_blue = new Scalar(140, 225, 225); //10 255 255

        Mat mask = new Mat();
        Core.inRange(roi_hsv, lower_blue, upper_blue, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        hierarchy.release();

        if(!contours.isEmpty()){
            double largest_area = 0;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > largest_area) {
                    largest_area = area;
                }
            }

            return largest_area;
        }
        else{
            return 0;
        }
    }
}