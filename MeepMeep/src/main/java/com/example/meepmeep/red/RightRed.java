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
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -66, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(10.5, -26.5, Math.toRadians(180)), Math.toRadians(90))
                                .waitSeconds(0.3)
                                // move away from pixel
                                // bring slides up here
                                .lineTo(new Vector2d(45, -29))
                                // drop
                                .waitSeconds(2)
                                // park
                                .forward(6)
                                .strafeLeft(10)
                                .splineToLinearHeading(new Pose2d(59, -60, Math.toRadians(180)), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity middleBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        // middle
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(12, -32, Math.toRadians(90)))
                                // intake push and drop pixel
                                .waitSeconds(0.3)
                                // move away from pixel
                                .lineTo(new Vector2d(12, -43))
                                // bring slides up here
                                .lineToSplineHeading(new Pose2d(45, -37, Math.toRadians(180)))
                                // drop
                                .waitSeconds(2)
                                // park
                                .forward(4)
                                .strafeLeft(4)
                                .splineToLinearHeading(new Pose2d(59, -60, Math.toRadians(180)), Math.toRadians(0))
                                .build()
                );


        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        // middle
                        drive.trajectorySequenceBuilder(new Pose2d(12, -66, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(13, -26.5, Math.toRadians(0)), Math.toRadians(90))
                                .waitSeconds(0.3)
                                //move away from pixel
                                .back(2)
                                // bring slides up
                                // drop
                                .strafeRight(22)
                                .lineToSplineHeading(new Pose2d(45, -42, Math.toRadians(180)))
                                .waitSeconds(2)
                                //park
                                .forward(5)
                                .strafeLeft(3)
                                .splineToLinearHeading(new Pose2d(59, -60, Math.toRadians(180)), Math.toRadians(0))
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