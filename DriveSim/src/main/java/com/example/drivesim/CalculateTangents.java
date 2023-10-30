package com.example.drivesim;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class CalculateTangents {
    public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.position.x - finalPosition.position.x;
        double yd = initialPosition.position.y - finalPosition.position.y;
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.x - finalPosition.x;
        double yd = initialPosition.y - finalPosition.y;
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Pose2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.position.x - finalPosition.x;
        double yd = initialPosition.position.y - finalPosition.y;
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.x - finalPosition.position.x;
        double yd = initialPosition.y - finalPosition.position.y;
        return Math.atan2(yd,xd) - Math.PI;
    }
}
