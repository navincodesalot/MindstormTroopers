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
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.625, -64.4, Math.toRadians(270)))
                                .strafeRight(3)
                                .lineToSplineHeading(new Pose2d(9.5, -32, Math.toRadians(180)))
                                .lineTo(new Vector2d(6, -32))

                                .lineToConstantHeading(new Vector2d(54, -31))

                                .forward(5)

                                .lineToLinearHeading(new Pose2d(49.5, -59, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity middleBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.625, -64.4, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(16, -23, Math.toRadians(180)))

                                .lineToConstantHeading(new Vector2d(54, -37))

                                .forward(5)

                                .lineToLinearHeading(new Pose2d(49.5, -59, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.625, -64.4, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(28.5, -28, Math.toRadians(180)))

                                .lineToConstantHeading(new Vector2d(54, -44))

                                .forward(5)

                                .lineToLinearHeading(new Pose2d(49.5, -59, Math.toRadians(180)))
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