package org.firstinspires.ftc.teamcode.newDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.usefuls.Math.T;

//the primary purpose of this will be to follow a point given by a Pose2d without using a PID
public class VectorDrive {
    DriveTrain drive;
    private double deltaX, deltaY, deltaR;
    private double xOut, yOut, rOut;
    private ElapsedTime headingTimer;
    private Pose2d currentPosition;
    private double lastHeading = 0, currentHeading = 0, currentTime = 0, lastTime = 0, turnVelocity = 0; //used for heading velocity
    private double followRadius;
    private double maxPower = 1;
    private double rawHeading;
    private double headingP = 1.3, headingD = 105;

    public VectorDrive(DriveTrain drive){
        this.drive = drive;
    }
    public void setFollowRadius(double followRadius){
        this.followRadius = followRadius;
    } //sets the follow radius
    public void setMaxPower(double maxPower){
        this.maxPower = maxPower;
    }//sets max power
    public void goToTarget(Pose2d targetPoint){
        lastHeading = currentHeading;

        //update our position
        drive.updatePos();
        currentPosition = drive.getGlobalPos();

        //deltas
        deltaX = targetPoint.getX() - currentPosition.getX();
        deltaY = targetPoint.getY() - currentPosition.getY();
        deltaR = targetPoint.getHeading() - currentHeading;

        xOut = deltaX;
        yOut = deltaY;
        //normalize it to a unit vector based on the follow radius
        xOut *= maxPower / followRadius;
        yOut *= maxPower / followRadius;

        //heading PD controller
        currentHeading = currentPosition.getHeading();
        lastTime = currentTime;
        currentTime = headingTimer.nanoseconds() / 1000000;
        turnVelocity = (currentHeading-lastHeading)/(currentTime-lastTime);
        rOut = -((targetPoint.getHeading() -  currentHeading) * headingP - turnVelocity * headingD);
        double rPower = rOut;

        //rotation matrix for x and y
        rawHeading = drive.getrRn();
        double xPower = (xOut * T.cos(rawHeading) - yOut * T.sin(rawHeading));
        double yPower = (xOut * T.sin(rawHeading) + yOut * T.cos(rawHeading));

        //if heading erorr is greater than 25 degrees, stop moving. if heading error exists, make moving a bit slower
        double zeroMoveAngle = Math.toRadians(25);
        double errorScale = 1 - (Math.abs(deltaR) / zeroMoveAngle);
        if(errorScale < 0) { errorScale = 0; }
        xPower *= errorScale;
        yPower *= errorScale;

        //finally setting the powers
        drive.setPowers(-xPower, -yPower, rPower);
    }

}
