package org.firstinspires.ftc.teamcode.usefuls.Math;

//T for Trigonometry !!
public class T {

    public static double hypot(double a, double b) {
        return Math.hypot(a,b);
    }
    //hypotenuse legnth






    //forward trig functions (outputs numbers)

    public static double sin(double rad) {
        return Math.sin(rad);
    }
    public static double sin(double opposite, double hypotenuse){
        return opposite/hypotenuse;
    }
    //sin

    public static double cos(double rad) {
        return Math.cos(rad);
    }
    public static double cos(double adjacent, double hypotenuse){
        return adjacent/hypotenuse;
    }
    //cos

    public static double tan(double rad) {
        return Math.tan(rad);
    }
    public static double tan(double opposite, double adjacent){
        return opposite/adjacent;
    }
    //tan







    //inverse trig functions (output angles)

    public static double arctan(double x) {
        return Math.atan(x);
    }
    //arc tangent, only 1 vector, and domain restriction from -pi/2 to pi/2

    public static double arctan2(double y, double x) {
        return Math.atan2(y,x);
    }
    //arc tangent but better, y,x, and domain restricted to -pi and pi

    public static double arccos(double v) {
        return Math.acos(v);
    }
    //arc cosine

    public static double arcsin(double v) {
        return Math.asin(v);
    }
    //arc sin





}