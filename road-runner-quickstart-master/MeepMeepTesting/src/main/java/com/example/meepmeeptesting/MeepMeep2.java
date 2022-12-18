package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        double blueX = 35;
        double redX = returnXCoord(blueX);

        double blueY = 61;
        double redY = returnYCoord(blueY);
        Pose2d startPose = new Pose2d(blueX, blueY, Math.toRadians(-90));

        int maxVel = 55, maxAccel = 45, trackWidth = 13;

        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        RoadRunnerBotEntity rightBlueBot = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(maxVel, maxAccel, 5.50, Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                                 drive.trajectorySequenceBuilder(startPose) // increment y to go further towards blue wall
                                .waitSeconds(1)
                                .lineTo(new Vector2d(35, 1.8)) // 2
                                .lineToSplineHeading(new Pose2d(77, 1.8, Math.toRadians(180))) // 1
                                .build()
                );
//                        drive.trajectorySequenceBuilder(new Pose2d(blueX, blueY, Math.toRadians(-90))) // increment y to go further towards blue wall
//                                .waitSeconds(0.5)
//                                .lineToSplineHeading(new Pose2d(32, 10, Math.toRadians(230))) // coterminal ang
//                                .waitSeconds(0.5)
//                                .lineToSplineHeading(new Pose2d(57, 11.5, Math.toRadians(0)))
//                                .waitSeconds(1)
//
//                                .lineToSplineHeading(new Pose2d(32, 10, Math.toRadians(230)))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(57, 11.5, Math.toRadians(0)))
//                                .waitSeconds(1)
//
//                                .lineToSplineHeading(new Pose2d(32, 10, Math.toRadians(230)))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(57, 11.5, Math.toRadians(0)))
//                                .waitSeconds(1)
//
//                                .lineToSplineHeading(new Pose2d(32, 10, Math.toRadians(230)))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(57, 11.5, Math.toRadians(0)))
//                                .waitSeconds(1)
//
//                                .lineToSplineHeading(new Pose2d(32, 10, Math.toRadians(230)))
//                                .waitSeconds(1)
//                                .lineToSplineHeading(new Pose2d(35, 13, Math.toRadians(0)))
//                                .build()
//                );
        RoadRunnerBotEntity leftBlueBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(maxVel, maxAccel, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(redX, blueY, Math.toRadians(-90))) // increment y to go further towards blue wall
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(-35, 10, Math.toRadians(-50))) // opp coterminal ang
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-57, 11.5, Math.toRadians(180)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-32, 10, Math.toRadians(-50)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-57, 11.5, Math.toRadians(180)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-32, 10, Math.toRadians(-50)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-57, 11.5, Math.toRadians(180)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-32, 10, Math.toRadians(-50)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-57, 11.5, Math.toRadians(180)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-32, 10, Math.toRadians(-50)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-35, 13, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity rightRedBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(maxVel, maxAccel, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(blueX, redY, Math.toRadians(90))) // increment y to go further towards blue wall
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(32, -10, Math.toRadians(-230))) // opp coterminal ang
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(57, -11.5, Math.toRadians(0)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(32, -10, Math.toRadians(-230)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(57, -11.5, Math.toRadians(0)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(32, -10, Math.toRadians(-230)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(57, -11.5, Math.toRadians(0)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(32, -10, Math.toRadians(-230)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(57, -11.5, Math.toRadians(0)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(32, -10, Math.toRadians(-230)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(35, -13, Math.toRadians(0)))
                                .build()
                );

        RoadRunnerBotEntity leftRedBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(maxVel, maxAccel, Math.toRadians(180), Math.toRadians(180), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(redX, redY, Math.toRadians(90)))
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-32, -10, Math.toRadians(50))) // coterminal ang
                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(-57, -11.5, Math.toRadians(180)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-32, -10, Math.toRadians(50)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-57, -11.5, Math.toRadians(180)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-32, -10, Math.toRadians(50)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-57, -11.5, Math.toRadians(180)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-32, -10, Math.toRadians(50)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-57, -11.5, Math.toRadians(180)))
                                .waitSeconds(1)

                                .lineToSplineHeading(new Pose2d(-32, -10, Math.toRadians(50)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-35, -13, Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rightBlueBot)
                .addEntity(leftBlueBot)
                .addEntity(rightRedBot)
                .addEntity(leftRedBot)
                .start();
    }

    public static double returnXCoord(double x) {
        return x * (-1);
    }

    public static double returnYCoord(double y) {
        return y * (-1);
    }

}

//
//.waitSeconds(0.5)
//        .splineToSplineHeading(new Pose2d(35.6,14.4, Math.toRadians(-126)), Math.toRadians(-126))// goes to junct
//        .splineToSplineHeading(new Pose2d(30.8,8.4, Math.toRadians(-134)), Math.toRadians(-134)) // goes to junct
//        .waitSeconds(1)
//        .splineToSplineHeading(new Pose2d(50,12.4, Math.toRadians(-3)), Math.toRadians(-3))
//        .splineToSplineHeading(new Pose2d(59.2,11.6, Math.toRadians(6)), Math.toRadians(6))// pickup
//        .waitSeconds(0.5) // pickup
//
//        .splineToSplineHeading(new Pose2d(54.4,12, Math.toRadians(-174)), Math.toRadians(-174))
//        .splineToSplineHeading(new Pose2d(42,10.8, Math.toRadians(-167)), Math.toRadians(-167))
//        .splineToSplineHeading(new Pose2d(29.6,8.4, Math.toRadians(-129)), Math.toRadians(-129)) // drops second
//        .waitSeconds(0.5)
//
//        .splineToSplineHeading(new Pose2d(50,12.4, Math.toRadians(-3)), Math.toRadians(-3))
//        .splineToSplineHeading(new Pose2d(59.2,11.6, Math.toRadians(6)), Math.toRadians(6))// pickup
//        .waitSeconds(0.5)
//
//        .splineToSplineHeading(new Pose2d(54.4,12, Math.toRadians(-174)), Math.toRadians(-174))
//        .splineToSplineHeading(new Pose2d(42,10.8, Math.toRadians(-167)), Math.toRadians(-167))
//        .splineToSplineHeading(new Pose2d(29.6,8.4, Math.toRadians(-129)), Math.toRadians(-129)) // drops third
//        .waitSeconds(0.5)
//
//        .splineToSplineHeading(new Pose2d(50,12.4, Math.toRadians(-3)), Math.toRadians(-3))
//        .splineToSplineHeading(new Pose2d(59.2,11.6, Math.toRadians(6)), Math.toRadians(6))// pickup
//        .waitSeconds(0.5)
//
//        .splineToSplineHeading(new Pose2d(54.4,12, Math.toRadians(-174)), Math.toRadians(-174))
//        .splineToSplineHeading(new Pose2d(42,10.8, Math.toRadians(-167)), Math.toRadians(-167))
//        .splineToSplineHeading(new Pose2d(29.6,8.4, Math.toRadians(-129)), Math.toRadians(-129)) // drops fourth
//        .waitSeconds(0.5)
//
//        .splineToSplineHeading(new Pose2d(50,12.4, Math.toRadians(-3)), Math.toRadians(-3))
//        .splineToSplineHeading(new Pose2d(59.2,11.6, Math.toRadians(6)), Math.toRadians(6))// pickup
//        .waitSeconds(0.5)
//
//        .splineToSplineHeading(new Pose2d(54.4,12, Math.toRadians(-174)), Math.toRadians(-174))
//        .splineToSplineHeading(new Pose2d(42,10.8, Math.toRadians(-167)), Math.toRadians(-167))
//        .splineToSplineHeading(new Pose2d(29.6,8.4, Math.toRadians(-129)), Math.toRadians(-129)) // drops fifth
//        .waitSeconds(0.5)
//
//
//        .build()