package org.firstinspires.ftc.teamcode.usefuls.Math;

public class vPoint {

    public double x, y;

    public vPoint(double x, double y){
        this.x = x;
        this.y = y;
    }

    public vPoint clone(){
        return new vPoint(x,y);
    }

}
