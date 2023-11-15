//package com.example.drivesim;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//public class DriveSim {
//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(800);
////        Pose2d initialPosition = new Pose2d(59, -38, Math.toRadians(0));
////        Vector2d secondPosition = new Vector2d(12,-38);
////        Pose2d thirdPosition = new Pose2d(35,-32,Math.toRadians(90));
////        Pose2d fourthPosition = new Pose2d(12,-50,Math.toRadians(-90));
////        Pose2d fifthPosition = new Pose2d(12,-55, Math.toRadians(-90));
////        Pose2d sixthPosition = new Pose2d(12,0, Math.toRadians(-90));
////        Pose2d seventhPosition = new Pose2d(15,11, Math.toRadians(-120));
//        Pose2d initialPosition = new Pose2d(59, -38, Math.toRadians(0));
//        Vector2d secondPosition = new Vector2d(12,-38);
//        Pose2d thirdPosition = new Pose2d(35,-32,Math.toRadians(90));
//        Pose2d fourthPosition = new Pose2d(12,-50,Math.toRadians(-90));
//        Pose2d fifthPosition = new Pose2d(12,-55, Math.toRadians(-90));
//        Pose2d sixthPosition = new Pose2d(12,0, Math.toRadians(-90));
//        Pose2d seventhPosition = new Pose2d(15,11, Math.toRadians(-120));
//
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(52.48291908330528, 70, Math.toRadians(180), Math.toRadians(180), 14.35)
//                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(initialPosition)
////                                .splineToSplineHeading(new Pose2d(-12.5, -60, Math.toRadians(-180)), Math.toRadians(180)) set x to 0
//////                                .splineToConstantHeading(new Vector2d(-12.5, -11.5), Math.toRadians(100))
//////                                .splineToConstantHeading(new Vector2d(-24.5, -11.5), Math.toRadians(70)) weird substation path
//////                                  .splineToConstantHeading(new Vector2d(-35.5, -35.5), Math.toRadians(170))
////////                                .splineTo(new Vector2d(-23.5, -12), Math.toRadians(180))
//////                                .lineToLinearHeading(new Pose2d(36, -2, Math.toRadians(-90))) right auton here
//////                                .lineToLinearHeading(new Pose2d(38, -10, Math.toRadians(0)))
//////                                .lineToLinearHeading(new Pose2d(11.5, -12, Math.toRadians(0))) park 1
//////                                .lineToLinearHeading(new Pose2d(36, -12.1, Math.toRadians(-90))) park 2
//////                                .lineToLinearHeading(new Pose2d(59, -12, Math.toRadians(0))) park 3
//////                                .lineToLinearHeading(new Pose2d(36, -18, Math.toRadians(-90)))
//////                                .splineToSplineHeading(new Pose2d(20, -14, Math.toRadians(0)), Math.toRadians(0)) goofy spline shit for right side
//////                                .splineTo(new Vector2d(24, -11), Math.toRadians(0))
//                                        .lineTo(secondPosition)
//                                        .splineToSplineHeading(thirdPosition, CalculateTangents.calculateTangent(secondPosition,thirdPosition)) //or y -47 and -45degree for down randomization and x 24 for straight
////                                .lineToLinearHeading(fourthPosition)
//                                        .splineToSplineHeading(fourthPosition, CalculateTangents.calculateTangent(thirdPosition,fourthPosition))
//                                        .lineTo(new Vector2d(fourthPosition.position.x,fourthPosition.position.y-10))
//                                        .lineToLinearHeading(sixthPosition)
//                                        .splineToSplineHeading(seventhPosition, CalculateTangents.calculateTangent(sixthPosition,seventhPosition))
//                                        .build()
//                );
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
//    }
//
//}

package com.example.drivesim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DriveSim {
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




    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        int propID = 2;
        Pose2d startPose = new Pose2d(-35, -65, Math.toRadians(-90));
        Pose2d leftProp = new Pose2d(-35, -29, Math.toRadians(-180));
        Pose2d rightProp = new Pose2d(-35, -29, Math.toRadians(0));
        Pose2d centerProp = new Pose2d(-35, -16, Math.toRadians(-90));
        Pose2d propDir[] = {leftProp, centerProp, rightProp};
        Pose2d afterPropID = new Pose2d(-35, -11.6, Math.toRadians(-90));
        Vector2d beforeYellow = new Vector2d(25, -11.6);
        double depoAngle[] = {Math.toRadians(-165),  Math.toRadians(-180),Math.toRadians(-195) };
        Pose2d yellowPixel = new Pose2d(35, -34, depoAngle[propID]);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .back(65-11.6)
                                .splineToSplineHeading(propDir[propID], CalculateTangents.calculateTangent(new Vector2d(-35, -11.6), propDir[propID]))
                                .splineToSplineHeading(afterPropID, CalculateTangents.calculateTangent(propDir[propID], afterPropID))
                                .lineTo(beforeYellow)
                                .splineToSplineHeading(yellowPixel, CalculateTangents.calculateTangent(beforeYellow, yellowPixel))
                                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}