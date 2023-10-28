package com.example.drivesim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.UMathKt;

public class DriveSim {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        int x =0, y=0;//arbitrary adjustment values
        int posNum = 0;//0, 1, 2, 3 clockwise from bottom left
        int mult[][] = {{1,1,1}, {1, -1, -1}, {-1, -1}, {-1, 1}};
        Pose2d initialPosition = new Pose2d(-35*mult[posNum][0]+x, -65*mult[posNum][1]+y, Math.toRadians(-90*mult[posNum][2]));
        Vector2d secondPosition = new Vector2d(-35*mult[posNum][0]+x,-11.6*mult[posNum][1]+y);
        Pose2d thirdPositionR = new Pose2d(-35*mult[posNum][0]+x,-29*mult[posNum][1]+y,Math.toRadians(0));//or -180
        Pose2d thirdPositionL = new Pose2d(-35*mult[posNum][0]+x, -29*mult[posNum][1]+y, Math.toRadians(-180*mult[posNum][2]));
        Vector2d thirdPositionF = new Vector2d(-35*mult[posNum][0]+x, -14*mult[posNum][1]+y);
        Pose2d fourthPosition = new Pose2d(-35*mult[posNum][0]+x,-11.6*mult[posNum][1]+y,Math.toRadians(-90*mult[posNum][2]));
        Pose2d fifthPosition = new Pose2d(-55*mult[posNum][0]+x,-11.6*mult[posNum][1]+y, Math.toRadians(-180*mult[posNum][2]));
        Pose2d sixthPosition = new Pose2d(5*mult[posNum][0]+x, -11.6*mult[posNum][1]+y, Math.toRadians(-180*mult[posNum][2]));
        Pose2d seventhPosition = new Pose2d(14*mult[posNum][0]+x, -16*mult[posNum][1]+y, Math.toRadians(-200*mult[posNum][2]));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 70, Math.toRadians(180), Math.toRadians(180), 14.35)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(initialPosition)
//                                .splineToSplineHeading(new Pose2d(-12.5, -60, Math.toRadians(-180)), Math.toRadians(180)) set x to 0
////                                .splineToConstantHeading(new Vector2d(-12.5, -11.5), Math.toRadians(100))
////                                .splineToConstantHeading(new Vector2d(-24.5, -11.5), Math.toRadians(70)) weird substation path
////                                  .splineToConstantHeading(new Vector2d(-35.5, -35.5), Math.toRadians(170))
//////                                .splineTo(new Vector2d(-23.5, -12), Math.toRadians(180))
////                                .lineToLinearHeading(new Pose2d(36, -2, Math.toRadians(-90))) right auton here
////                                .lineToLinearHeading(new Pose2d(38, -10, Math.toRadians(0)))
////                                .lineToLinearHeading(new Pose2d(11.5, -12, Math.toRadians(0))) park 1
////                                .lineToLinearHeading(new Pose2d(36, -12.1, Math.toRadians(-90))) park 2
////                                .lineToLinearHeading(new Pose2d(59, -12, Math.toRadians(0))) park 3
////                                .lineToLinearHeading(new Pose2d(36, -18, Math.toRadians(-90)))
////                                .splineToSplineHeading(new Pose2d(20, -14, Math.toRadians(0)), Math.toRadians(0)) goofy spline shit for right side
////                                .splineTo(new Vector2d(24, -11), Math.toRadians(0))
                                        .lineTo(secondPosition)
                                        .splineToSplineHeading(thirdPositionR, CalculateTangents.calculateTangent(secondPosition,thirdPositionR)) //or y -47 and -45degree for down randomization and x 24 for straight
//                                .lineToLinearHeading(fourthPosition)
                                        .splineToSplineHeading(fourthPosition, CalculateTangents.calculateTangent(thirdPositionR,fourthPosition))
                                        .lineToLinearHeading(fifthPosition)
                                        .lineToLinearHeading(sixthPosition)
                                        .splineToSplineHeading(seventhPosition, CalculateTangents.calculateTangent(sixthPosition, seventhPosition))
//                                        .lineTo(sixthPosition)
//                                        .strafeTo(seventhPosition)
//                                        .splineToSplineHeading(fifthPosition, CalculateTangents.calculateTangent(fourthPosition,fifthPosition))
//                                        .lineTo(new Vector2d(fourthPosition.getX(),fourthPosition.getY()-10))
//                                        .lineToLinearHeading(sixthPosition)
//                                        .splineToSplineHeading(seventhPosition, CalculateTangents.calculateTangent(sixthPosition,seventhPosition))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}