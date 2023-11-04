package com.example.drivesim;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.UMathKt;

public class DriveSim {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        int x =0, y=0;//arbitrary adjustment values
        int posNum = 1;//0, 1, 2, 3 clockwise from bottom left
        int mult[][] = {{1,1,1}, {1, -1,1}, {-1, -1}, {-1, 1}};
        Pose2d initialPosition = new Pose2d(-35*mult[posNum][0]+x, -65*mult[posNum][1]+y, Math.toRadians(-90*mult[posNum][2]));
        double secondPosition = -11.6*mult[posNum][1]+y;
        Vector2d secondPositionV = new Vector2d(-35*mult[posNum][0]+x,-11.6*mult[posNum][1]+y);
        Pose2d thirdPositionR = new Pose2d(-35*mult[posNum][0]+x,-29*mult[posNum][1]+y,Math.toRadians(0));//or -180
        Pose2d thirdPositionL = new Pose2d(-35*mult[posNum][0]+x, -29*mult[posNum][1]+y, Math.toRadians(-180*mult[posNum][2]));
        Vector2d thirdPositionF = new Vector2d(-35*mult[posNum][0]+x, -14*mult[posNum][1]+y);
        Pose2d fourthPosition = new Pose2d(-35*mult[posNum][0]+x,-11.6*mult[posNum][1]+y,Math.toRadians(-90*mult[posNum][2]));
        Pose2d fifthPosition = new Pose2d(-55*mult[posNum][0]+x,-11.6*mult[posNum][1]+y,Math.toRadians(-180*mult[posNum][2]));
        Pose2d sixthPosition = new Pose2d(5*mult[posNum][0]+x, -11.6*mult[posNum][1]+y,Math.toRadians(-180*mult[posNum][2]));
        Pose2d seventhPosition = new Pose2d(14*mult[posNum][0]+x, -16*mult[posNum][1]+y, Math.toRadians(-200*mult[posNum][2]));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 70, Math.toRadians(180), Math.toRadians(180), 14.35).build();
                myBot.runAction(myBot.getDrive().actionBuilder(initialPosition)
                                        .lineToY(secondPosition)
                                        .splineToSplineHeading(thirdPositionR, CalculateTangents.calculateTangent(secondPositionV,thirdPositionR)) //or y -47 and -45degree for down randomization and x 24 for straight

                                        .splineToSplineHeading(fourthPosition, CalculateTangents.calculateTangent(thirdPositionR,fourthPosition))
                        .turn(-Math.PI/2).lineToX(-55*mult[posNum][0]).lineToX(5*mult[posNum][0]).splineToSplineHeading(seventhPosition,CalculateTangents.calculateTangent(sixthPosition, seventhPosition)).build());





        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}