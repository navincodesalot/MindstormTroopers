package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.graalvm.compiler.nodes.debug.SpillRegistersNode;

public class MeepMeep2 {
    public static void main(String[] args) {
        double pickX = 42, pickY = 8, pickHead = -149;
        double dropX = 50, dropY = 12, dropHead = 0;
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d leftBlueStartPose = new Pose2d(35, 61, Math.toRadians(-90));
        Pose2d rightBlueStartPose = new Pose2d(returnX(35), 61, Math.toRadians(-90));

        Pose2d leftRedStartPose = new Pose2d(35, returnY(61), Math.toRadians(-270));
        Pose2d rightRedStartPose = new Pose2d(returnX(35), returnY(61), Math.toRadians(-270));

        int maxVel = 30, maxAccel = 30, trackWidth = 13;

        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        RoadRunnerBotEntity leftBlue = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(maxVel, maxAccel, 4.1715, Math.toRadians(60), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(leftBlueStartPose) // increment y to go further towards blue wall
                                .waitSeconds(1) // detect
                                .lineTo(new Vector2d(35, 8))
                                .addTemporalMarker(2, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(pickHead)))
                                .waitSeconds(1) //bucket drop
                                .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(dropHead)))
                                .addTemporalMarker(6.4, () -> {
                                    //slide.setPower(0)
                                    //arm.setPower
                                })
                                .waitSeconds(5)
                                //claw.setPosition
                                //arm back up
                                // claw open
                                //arm station
                                .addTemporalMarker(13, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(pickHead)))
                                .waitSeconds(1) //bucket drop
                                .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(dropHead)))
                                .addTemporalMarker(6.4, () -> {
                                    //slide.setPower(0)
                                    //arm.setPower
                                })
                                .waitSeconds(5)
                                //claw.setPosition
                                //arm back up
                                // claw open
                                //arm station
                                .addTemporalMarker(13, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(pickHead)))
                                .waitSeconds(1) //bucket drop
                                .addTemporalMarker(26, () -> {
                                    //slide.setPower(0)
                                })
                                .lineToSplineHeading(new Pose2d(35, 12.5, Math.toRadians(-90)))

                                .build()
                );
        RoadRunnerBotEntity rightBlue = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(maxVel, maxAccel, 4.1715, Math.toRadians(60), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(rightBlueStartPose) // increment y to go further towards blue wall
                                .waitSeconds(1) // detect
                                .lineTo(new Vector2d(returnX(35), 8))
                                .addTemporalMarker(2, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(returnX(pickX), pickY, Math.toRadians(returnHead(pickHead))))
                                .waitSeconds(1) //bucket drop
                                .lineToSplineHeading(new Pose2d(returnX(dropX), dropY, Math.toRadians(returnHead(dropHead))))
                                .addTemporalMarker(6.4, () -> {
                                    //slide.setPower(0)
                                    //arm.setPower
                                })
                                .waitSeconds(5)
                                //claw.setPosition
                                //arm back up
                                // claw open
                                //arm station
                                .addTemporalMarker(13, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(returnX(pickX), pickY, Math.toRadians(returnHead(pickHead))))
                                .waitSeconds(1) //bucket drop
                                .lineToSplineHeading(new Pose2d(returnX(dropX), dropY, Math.toRadians(returnHead(dropHead))))
                                .addTemporalMarker(6.4, () -> {
                                    //slide.setPower(0)
                                    //arm.setPower
                                })
                                .waitSeconds(5)
                                //claw.setPosition
                                //arm back up
                                // claw open
                                //arm station
                                .addTemporalMarker(13, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(returnX(pickX), pickY, Math.toRadians(returnHead(pickHead))))
                                .waitSeconds(1) //bucket drop
                                .addTemporalMarker(26, () -> {
                                    //slide.setPower(0)
                                })
                                .lineToSplineHeading(new Pose2d(returnX(35), 12.5, Math.toRadians(-90)))

                                .build()
                );
        RoadRunnerBotEntity leftRed = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(maxVel, maxAccel, 4.1715, Math.toRadians(60), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(leftRedStartPose) // increment y to go further towards blue wall
                                .waitSeconds(1) // detect
                                .lineTo(new Vector2d(35, returnY(8)))
                                .addTemporalMarker(2, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(pickX, returnY(pickY), Math.toRadians(returnHead(pickHead, 1))))
                                .waitSeconds(1) //bucket drop
                                .lineToSplineHeading(new Pose2d(dropX, returnY(dropY), Math.toRadians(returnHead(dropHead, 1))))
                                .addTemporalMarker(6.4, () -> {
                                    //slide.setPower(0)
                                    //arm.setPower
                                })
                                .waitSeconds(5)
                                //claw.setPosition
                                //arm back up
                                // claw open
                                //arm station
                                .addTemporalMarker(13, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(pickX, returnY(pickY), Math.toRadians(returnHead(pickHead, 1))))
                                .waitSeconds(1) //bucket drop
                                .lineToSplineHeading(new Pose2d(dropX, returnY(dropY), Math.toRadians(returnHead(dropHead, 1))))
                                .addTemporalMarker(6.4, () -> {
                                    //slide.setPower(0)
                                    //arm.setPower
                                })
                                .waitSeconds(5)
                                //claw.setPosition
                                //arm back up
                                // claw open
                                //arm station
                                .addTemporalMarker(13, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(pickX, returnY(pickY), Math.toRadians(returnHead(pickHead, 1))))
                                .waitSeconds(1) //bucket drop
                                .addTemporalMarker(26, () -> {
                                    //slide.setPower(0)
                                })
                                .lineToSplineHeading(new Pose2d(35, returnY(12.5), Math.toRadians(returnHead(90, 1))))

                                .build()
                );
        RoadRunnerBotEntity rightRed = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(maxVel, maxAccel, 4.1715, Math.toRadians(60), trackWidth)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(rightRedStartPose) // increment y to go further towards blue wall
                                .waitSeconds(1) // detect
                                .lineTo(new Vector2d(returnX(35), returnY(8)))
                                .addTemporalMarker(2, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(returnX(pickX), returnY(pickY), Math.toRadians(31)))
                                .waitSeconds(1) //bucket drop
                                .lineToSplineHeading(new Pose2d(returnX(dropX), returnY(dropY), Math.toRadians(returnHead(dropHead, "s"))))
                                .addTemporalMarker(6.4, () -> {
                                    //slide.setPower(0)
                                    //arm.setPower
                                })
                                .waitSeconds(5)
                                //claw.setPosition
                                //arm back up
                                // claw open
                                //arm station
                                .addTemporalMarker(13, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(returnX(pickX), returnY(pickY), Math.toRadians(31)))
                                .waitSeconds(1) //bucket drop
                                .lineToSplineHeading(new Pose2d(returnX(dropX), returnY(dropY), Math.toRadians(returnHead(dropHead, "s"))))
                                .addTemporalMarker(6.4, () -> {
                                    //slide.setPower(0)
                                    //arm.setPower
                                })
                                .waitSeconds(5)
                                //claw.setPosition
                                //arm back up
                                // claw open
                                //arm station
                                .addTemporalMarker(13, () -> {
                                    //slide.setPower
                                })
                                .lineToSplineHeading(new Pose2d(returnX(pickX), returnY(pickY), Math.toRadians(31)))
                                .waitSeconds(1) //bucket drop
                                .addTemporalMarker(26, () -> {
                                    //slide.setPower(0)
                                })
                                .lineToSplineHeading(new Pose2d(returnX(35), returnY(12.5), Math.toRadians(90)))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(leftBlue)
                .addEntity(rightBlue)
                .addEntity(leftRed)
                .addEntity(rightRed)
                .start();
    }

    public static double returnX(double x) {
        return x * (-1);
    }
    public static double returnHead(double h) { h = Math.abs(h); return h -= 180; }
    public static double returnHead(double h, int i) { h = Math.abs(h); return h -= 360; }
    public static double returnHead(double h, String s) { h = Math.abs(h); return h -= 180; }
    public static double returnY(double y) { return y * (-1); }
}
