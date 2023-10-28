package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class mathTest extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

    }
    int halfField = 72;//length of half field
    public double degreeToBoard(double x, double z){
        double adjacent = (halfField-x)+(13.5-(z/Math.tan(Math.PI/3)));
        double angleOfElevation = Math.atan(z/adjacent);
        angleOfElevation = (angleOfElevation/Math.PI)*180;
        return angleOfElevation;
    }

    public double xCoordOfBoard(double z){
        double x = halfField-13.5+(z/Math.tan(Math.PI/3));
        return x;
    }

}
