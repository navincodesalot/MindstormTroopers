package com.example.meepmeep.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.example.meepmeep.Constraints;

public class RightRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        // middle todo: (12, -70)
                        drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                                .splineTo(new Vector2d(0, -40), Math.toRadians(180))
                                .waitSeconds(0.3)
                                //move away from pixel
                                .lineTo(new Vector2d(0, -44))
                                // bring slides up here
                                .lineTo(new Vector2d(45, -29))
                                // drop
                                .waitSeconds(2)
                                .lineTo(new Vector2d(25, -55))
                                .lineTo(new Vector2d(59, -60))
                                .build()
                );

        RoadRunnerBotEntity middleBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        // middle
                        drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(12, -33, Math.toRadians(180)))
                                .waitSeconds(0.3)
                                //move away from pixel
                                .lineTo(new Vector2d(12, -34))
                                // bring slides up here
                                .lineToSplineHeading(new Pose2d(45, -35, Math.toRadians(180)))
                                // drop
                                .waitSeconds(2)
                                .splineToLinearHeading(new Pose2d(59, -60, Math.toRadians(180)), Math.toRadians(0))
//                                .lineTo(new Vector2d(25, -55))
//                                .lineTo(new Vector2d(59, -60))
                                .build()
                );


        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        // middle
                        drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(22, -40, Math.toRadians(180)))
                                .waitSeconds(0.3)
                                //move away from pixel
                                .lineTo(new Vector2d(22, -44))
                                // bring slides up here
                                .lineToSplineHeading(new Pose2d(45, -42, Math.toRadians(180)))
                                // drop
                                .waitSeconds(2)
                                .lineTo(new Vector2d(25, -55))
                                .lineTo(new Vector2d(59, -60))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(leftBot)
//                .addEntity(middleBot)
               .addEntity(rightBot)
                .start();
    }
}