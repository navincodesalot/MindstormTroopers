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
                                drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(-46, -39, Math.toRadians(90)))
                                        .back(4)
                                        .strafeRight(14)
                                        .lineToConstantHeading(new Vector2d(-36, -10))
                                        .lineToSplineHeading(new Pose2d(-63, -12, Math.toRadians(180)))
                                        .waitSeconds(2)
                                        .lineToSplineHeading(new Pose2d(35, 0, Math.toRadians(180)))
                                        .lineToConstantHeading(new Vector2d(56, -28))
                                        .waitSeconds(2)
                                        .lineToLinearHeading(new Pose2d(49.5, -15, Math.toRadians(180)))
                                        .build()
                );

        RoadRunnerBotEntity middleBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(-43, -21.5, Math.toRadians(0)))
                                .waitSeconds(0.3)
                                .lineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(180)))
                                .waitSeconds(2)
                                .lineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(56, -34))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(49.5, -15, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(270)))
                                .strafeRight(3)
                                .lineToSplineHeading(new Pose2d(-32, -23.5, Math.toRadians(0)))
                                .lineTo(new Vector2d(-46, -23.5))
                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(180)))
                                .waitSeconds(2)
                                .lineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(56, -41.5))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(49.5, -15, Math.toRadians(180)))
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