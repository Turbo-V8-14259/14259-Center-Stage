package org.firstinspires.ftc.teamcode.vision;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.CvType;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

public class PatternDetectionTest extends OpenCvPipeline
{

    public static String color = "BLUE";
    public boolean detected = false;
    public boolean detectDetected = false;

    public static enum ObjectPosition{
        LEFT,
        MIDDLE,
        RIGHT
    }

    public double x;
    public double y;

    public static ObjectPosition op; //default

    public Mat processFrame(Mat input)
    {

        Scalar lowHSVRed = new Scalar(0,100,20); // lower bound HSV for red 0 100 20
        Scalar highHSVRed = new Scalar(10, 255, 255); // higher bound HSV for red 10 255 255

        Scalar lowHSVBlue = new Scalar(75, 30, 10); // lower bound HSV for blue 110 100 20
        Scalar highHSVBlue = new Scalar(120, 255, 255); // higher bound HSV for blue 130 255 255

        //color ---------------------------------------------------
        Mat imgColor = new Mat();
        Imgproc.cvtColor(input, imgColor, Imgproc.COLOR_RGB2HSV);
        List<Mat> b = new ArrayList<Mat>();
        if(color.equals("BLUE")){
            Core.split(imgColor, b);
            Core.add(b.get(2), new Scalar(20), b.get(2));
            Core.merge(b,imgColor);
        }

        Mat thresh = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(8, 8));

        if(color.equals("RED")){
            Core.inRange(imgColor, lowHSVRed, highHSVRed, thresh);
        }
        else if (color.equals("BLUE")) {
            Core.inRange(imgColor, lowHSVBlue, highHSVBlue, thresh);
        }
        Imgproc.erode(thresh, thresh, kernel);
        Imgproc.dilate(thresh, thresh, kernel);

        kernel.release();

        Mat imgIn = new Mat();
        Imgproc.cvtColor(imgColor,imgColor, Imgproc.COLOR_HSV2RGB);
        Core.bitwise_and(imgColor, imgColor, imgIn, thresh);

        Mat a = new Mat();
        imgIn.copyTo(a);

        imgColor.release();

        //pattern ---------------------------------------------------------
        Mat template = new Mat();
        if(color.equals("RED")){
            template = Imgcodecs.imread(Environment.getExternalStorageDirectory().getPath() + "/FIRST/RedCone.jpg");
        }
        else if (color.equals("BLUE")) {
            template = Imgcodecs.imread(Environment.getExternalStorageDirectory().getPath() + "/FIRST/BlueCone.jpg");
        }


        Size s = new Size(100,150);
        Imgproc.resize(template, template, s);

        Imgproc.cvtColor(imgIn, imgIn, Imgproc.COLOR_RGB2GRAY);
        Imgproc.cvtColor(template, template, Imgproc.COLOR_RGB2GRAY);

        int result_cols = imgIn.cols() - template.cols() + 1;
        int result_rows = imgIn.rows() - template.rows() + 1;

        Mat output = new Mat(result_rows, result_cols, CvType.CV_32FC1);
        Imgproc.matchTemplate(imgIn,template,output,Imgproc.TM_CCOEFF_NORMED);

        imgIn.release();

        //Result ---------------------------------------------

        Core.MinMaxLocResult Result = Core.minMaxLoc(output);

        output.release();

        Point botRight = Result.maxLoc;
        Point topLeft = Result.minLoc;

        x = botRight.x + template.cols();
        y = botRight.y + template.rows();

        Point center = new Point((x + botRight.x)/2,(y + botRight.y)/2);
        Point center2 = new Point((topLeft.x + botRight.x)/2,(topLeft.y + botRight.y)/2);


        double[] value = thresh.get((int)center.y, (int)center.x);
        Imgproc.circle(a, center, 1, new Scalar(0,255,255));


        if(value == null){
        }
        else{
            if(value[0] > 1){
                Imgproc.rectangle(a, botRight, new Point(x,y), new Scalar(0,255,255), 2, 8, 0);   //input
                detected = true;
                detectDetected = true;
            }
            if(value[0] == 0){
                double[] sideRight = thresh.get((int)center.y, (int)center.x + 8);
                double[] sideLeft = thresh.get((int)center.y, (int)center.x - 8);
                if(sideRight[0] > 1 || sideLeft[0] > 1){
                    Imgproc.rectangle(a, botRight, new Point(x,y), new Scalar(0,255,255), 2, 8, 0);   //input
                    detected = true;
                    detectDetected = true;
                }
            }
        }
        int line = 150;

        if(center.x >= line){
            op = ObjectPosition.LEFT;
            //left
        }
        if(center.x < line){
            op = ObjectPosition.RIGHT;
            //right
        }
        if(!detected){
            op = ObjectPosition.MIDDLE;
        }

        thresh.release();
        return a;
    }

    public ObjectPosition getLocation(){
        return op;
    }

    public static Blue.Location convertB(ObjectPosition op){
        Blue.Location bl = Blue.Location.RIGHT;
        if(op == ObjectPosition.LEFT){
            bl = Blue.Location.LEFT;
        }
        if(op == ObjectPosition.RIGHT){
            bl = Blue.Location.RIGHT;
        }
        if(op == ObjectPosition.MIDDLE){
            bl = Blue.Location.MIDDLE;
        }
        return bl;
    }

    public static Red.Location convertR(ObjectPosition op){
        Red.Location r = Red.Location.RIGHT;
        if(op == ObjectPosition.LEFT){
            r = Red.Location.LEFT;
        }
        if(op == ObjectPosition.RIGHT){
            r = Red.Location.RIGHT;
        }
        if(op == ObjectPosition.MIDDLE){
            r = Red.Location.MIDDLE;
        }
        return r;
    }
}