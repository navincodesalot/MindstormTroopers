package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(63, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.6,68.4, Math.toRadians(-91)))
                                .waitSeconds(0.5)
                                .splineToSplineHeading(new Pose2d(35.6,14.4, Math.toRadians(-126)), Math.toRadians(-126))// goes to junct
                                .splineToSplineHeading(new Pose2d(30.8,8.4, Math.toRadians(-134)), Math.toRadians(-134)) // goes to junct
                                .waitSeconds(1)
                                .splineToSplineHeading(new Pose2d(50,12.4, Math.toRadians(-3)), Math.toRadians(-3))
                                .splineToSplineHeading(new Pose2d(59.2,11.6, Math.toRadians(6)), Math.toRadians(6))// pickup
                                .waitSeconds(0.5) // pickup

                                .splineToSplineHeading(new Pose2d(54.4,12, Math.toRadians(-174)), Math.toRadians(-174))
                                .splineToSplineHeading(new Pose2d(42,10.8, Math.toRadians(-167)), Math.toRadians(-167))
                                .splineToSplineHeading(new Pose2d(29.6,8.4, Math.toRadians(-129)), Math.toRadians(-129)) // drops second
                                .waitSeconds(0.5)

                                .splineToSplineHeading(new Pose2d(50,12.4, Math.toRadians(-3)), Math.toRadians(-3))
                                .splineToSplineHeading(new Pose2d(59.2,11.6, Math.toRadians(6)), Math.toRadians(6))// pickup
                                .waitSeconds(0.5)

                                .splineToSplineHeading(new Pose2d(54.4,12, Math.toRadians(-174)), Math.toRadians(-174))
                                .splineToSplineHeading(new Pose2d(42,10.8, Math.toRadians(-167)), Math.toRadians(-167))
                                .splineToSplineHeading(new Pose2d(29.6,8.4, Math.toRadians(-129)), Math.toRadians(-129)) // drops third
                                .waitSeconds(0.5)

                                .splineToSplineHeading(new Pose2d(50,12.4, Math.toRadians(-3)), Math.toRadians(-3))
                                .splineToSplineHeading(new Pose2d(59.2,11.6, Math.toRadians(6)), Math.toRadians(6))// pickup
                                .waitSeconds(0.5)

                                .splineToSplineHeading(new Pose2d(54.4,12, Math.toRadians(-174)), Math.toRadians(-174))
                                .splineToSplineHeading(new Pose2d(42,10.8, Math.toRadians(-167)), Math.toRadians(-167))
                                .splineToSplineHeading(new Pose2d(29.6,8.4, Math.toRadians(-129)), Math.toRadians(-129)) // drops fourth
                                .waitSeconds(0.5)

                                .splineToSplineHeading(new Pose2d(50,12.4, Math.toRadians(-3)), Math.toRadians(-3))
                                .splineToSplineHeading(new Pose2d(59.2,11.6, Math.toRadians(6)), Math.toRadians(6))// pickup
                                .waitSeconds(0.5)

                                .splineToSplineHeading(new Pose2d(54.4,12, Math.toRadians(-174)), Math.toRadians(-174))
                                .splineToSplineHeading(new Pose2d(42,10.8, Math.toRadians(-167)), Math.toRadians(-167))
                                .splineToSplineHeading(new Pose2d(29.6,8.4, Math.toRadians(-129)), Math.toRadians(-129)) // drops fifth
                                .waitSeconds(0.5)


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}