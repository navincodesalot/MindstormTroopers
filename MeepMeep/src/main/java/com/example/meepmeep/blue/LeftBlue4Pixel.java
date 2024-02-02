package com.example.meepmeep.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.example.meepmeep.Constraints;

public class LeftBlue4Pixel {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.625, 64.4, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(25.5, 17.5, Math.toRadians(112))) // drop first

                                .back(3)
                                .lineToSplineHeading(new Pose2d(54, 45, Math.toRadians(180))) // go to backdrop

                                .lineToLinearHeading(new Pose2d(49.5, 59, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity middleBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.625, 64.4, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(22, 27.5, Math.toRadians(200))) // ground pixel

                                .lineToSplineHeading(new Pose2d(54, 36, Math.toRadians(180))) // go to backdrop

//                                .forward(5) // back up only if i park

                                .lineToConstantHeading(new Vector2d(45,35)) // localize

                                .splineToConstantHeading(new Vector2d(18,35), Math.toRadians(180)) // cross truss
                                .splineToConstantHeading(new Vector2d(-38,36), Math.toRadians(180))

                                .lineToConstantHeading(new Vector2d(-63,35.65)) // go to stack

                                .back(1.5) // move back

                                .splineToConstantHeading(new Vector2d(45,35), Math.toRadians(180)) // cross truss again for whites

                                .build()
                );

        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.625, 64.4, Math.toRadians(90)))
                                .strafeRight(3)
                                .lineToSplineHeading(new Pose2d(7.5, 27, Math.toRadians(180)))

                                .lineToConstantHeading(new Vector2d(54, 31.5))

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