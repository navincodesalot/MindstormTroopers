package com.example.meepmeep.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.example.meepmeep.Constraints;

public class LeftBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(33.5, 23, Math.toRadians(180)))
                                .waitSeconds(0.3)
                                .lineToConstantHeading(new Vector2d(54, 43.5))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(49.5, 59, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity middleBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(16, 24, Math.toRadians(180)))
                                .waitSeconds(0.3)
                                .lineToConstantHeading(new Vector2d(54, 36))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(49.5, 59, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 64, Math.toRadians(90)))
                                .strafeRight(3)
                                .lineToSplineHeading(new Pose2d(16, 26, Math.toRadians(180)))
                                .waitSeconds(0.3)
                                .lineTo(new Vector2d(1, 26))
                                .lineToConstantHeading(new Vector2d(54, 30))
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(49.5, 59, Math.toRadians(180)))
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