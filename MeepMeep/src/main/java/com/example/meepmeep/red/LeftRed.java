package com.example.meepmeep.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.example.meepmeep.Constraints;

public class LeftRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-34, -25.5, Math.toRadians(180)))
                                        .waitSeconds(0.3)
                                        .lineToConstantHeading(new Vector2d(-48, -12))
                                        .waitSeconds(2)
                                        .lineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)))
                                        .lineToConstantHeading(new Vector2d(53, -31))
                                        .waitSeconds(2)
                                        .lineToLinearHeading(new Pose2d(49, -15, Math.toRadians(180)))
                                        .build()
                );

        RoadRunnerBotEntity middleBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(-43, -25.5, Math.toRadians(0)))
                                .waitSeconds(0.3)
                                .lineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(180)))
                                .waitSeconds(2)
                                .lineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(53, -37))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(49, -15, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(270)))
                                .strafeRight(2)
                                .lineToSplineHeading(new Pose2d(-31, -25.5, Math.toRadians(0)))
                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(180)))
                                .waitSeconds(2)
                                .lineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(53, -43))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(49, -15, Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(leftBot)
                .addEntity(middleBot)
//               .addEntity(rightBot)
                .start();
    }
}