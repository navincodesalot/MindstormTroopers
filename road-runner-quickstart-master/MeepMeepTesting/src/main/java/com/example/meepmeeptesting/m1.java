package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeep2.returnHead;
import static com.example.meepmeeptesting.MeepMeep2.returnX;
import static com.example.meepmeeptesting.MeepMeep2.returnY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class m1 {
    public static void main(String[] args) {
        double pickX = 40, pickY = 8, pickHead = -153;
        double dropX = 50, dropY = 12, dropHead = 0;
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d leftBlueStartPose = new Pose2d(35, 61, Math.toRadians(-90));
        Pose2d rightBlueStartPose = new Pose2d(returnX(35), 61, Math.toRadians(-90));

        Pose2d leftRedStartPose = new Pose2d(35, returnY(61), Math.toRadians(-270));
        Pose2d rightRedStartPose = new Pose2d(returnX(35), returnY(61), Math.toRadians(-270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(rightBlueStartPose)
                                .waitSeconds(1) // detect
                                .lineTo(new Vector2d(returnX(35), 3))
                                .lineTo(new Vector2d(returnX(35), 8))
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                    slideTarget = sHigh;
                                })
                                .lineToSplineHeading(new Pose2d(returnX(pickX), pickY, Math.toRadians(returnHead(pickHead, 1))))
                                .addSpatialMarker(new Vector2d(returnX(pickX), pickY), () -> {
//                    bclaw.setPosition(0.92);
                                })
                                .waitSeconds(2.5)
                                .addSpatialMarker(new Vector2d(returnX(pickX), pickY), () -> {
//                    bclaw.setPosition(0); //bucket pickup
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}