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
                        drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(270)))
                                .strafeLeft(2)
                                .lineToSplineHeading(new Pose2d(10.5, -26.5, Math.toRadians(182)))
                                .waitSeconds(0.3)
                                // move away from pixel
                                // bring slides up here
                                .lineTo(new Vector2d(45, -29))
                                // drop
                                .waitSeconds(2)
                                // park
//                                .strafeLeft(10)
                                .lineToLinearHeading(new Pose2d(47, -60, Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity middleBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        // middle
                        drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(23, -23, Math.toRadians(182)))
                                // intake push and drop pixel
                                .waitSeconds(0.3)
                                // bring slides up here
                                .lineToConstantHeading(new Vector2d(47, -35))
                                // drop
                                .waitSeconds(2)
                                // park
                                .lineToLinearHeading(new Pose2d(47, -60, Math.toRadians(180)))
                                .build()
                );


        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(Constraints.MAX_VEL, Constraints.MAX_ACCEL, Constraints.MAX_ANG_VEL,
                        Constraints.MAX_ANG_ACCEL, Constraints.TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        // middle
                        drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(35, -29, Math.toRadians(180)))
                                .waitSeconds(0.3)
                                .lineToConstantHeading(new Vector2d(47, -44))
                                .waitSeconds(2)
                                //park
                                .lineToLinearHeading(new Pose2d(47, -60, Math.toRadians(180)))
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