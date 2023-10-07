package org.firstinspires.ftc.teamcode.usefuls.Math;

public class M {
    public static double lerp(double a, double b, double w) {
        return a * (1.0 - w) + b * w;
    }
    //Linear interpolation

    public static double clamp(double x, double a, double b) {
        return Math.min(Math.max(x, a), b);
    }
    //Max
    public static double normalize(double x, double a, double b) {
        return (x - a) / (b - a);
    }
    //Given
}