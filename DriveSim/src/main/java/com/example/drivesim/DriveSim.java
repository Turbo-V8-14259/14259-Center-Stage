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

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DriveSim {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}