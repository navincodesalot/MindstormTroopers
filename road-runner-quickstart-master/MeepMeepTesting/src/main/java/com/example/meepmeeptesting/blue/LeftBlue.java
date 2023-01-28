package com.example.meepmeeptesting.blue;

import static com.example.meepmeeptesting.BaseMeep.dropHead;
import static com.example.meepmeeptesting.BaseMeep.dropX;
import static com.example.meepmeeptesting.BaseMeep.dropY;
import static com.example.meepmeeptesting.BaseMeep.park;
import static com.example.meepmeeptesting.BaseMeep.pickHead;
import static com.example.meepmeeptesting.BaseMeep.pickHead1;
import static com.example.meepmeeptesting.BaseMeep.pickX;
import static com.example.meepmeeptesting.BaseMeep.pickX1;
import static com.example.meepmeeptesting.BaseMeep.pickY;
import static com.example.meepmeeptesting.BaseMeep.pickY1;
import static com.example.meepmeeptesting.BaseMeep.leftBlueStartPose;
import static com.example.meepmeeptesting.BaseMeep.maxAccel;
import static com.example.meepmeeptesting.BaseMeep.maxVel;
import static com.example.meepmeeptesting.BaseMeep.returnHead;
import static com.example.meepmeeptesting.BaseMeep.trackWidth;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class LeftBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity b;
        if (park == 1) {
            b = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(maxVel, maxAccel, 4.1715, Math.toRadians(60), trackWidth)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(leftBlueStartPose)
                                            .lineTo(new Vector2d(35, 3))
                                            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                                slideTarget = sHigh;
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aStatic;
//                                                }
                                            })
                                            .lineToSplineHeading(new Pose2d(pickX1, pickY1, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX1, pickY1), () -> {
//                                               if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               bclaw.setPosition(0);
//                                               slideTarget = sLow;
                                            })
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               bclaw.setPosition(0);
//                                                slideTarget = sLow;
                                            })
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)

//                                           PARK
                                            .lineToSplineHeading(new Pose2d(12, 13, Math.toRadians(-90)))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                slideTarget = sLow;
                                            })
                                            .build()
                    );
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(b)
                    .start();
        } else if (park == 2) {
            b = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(maxVel, maxAccel, 4.1715, Math.toRadians(60), trackWidth)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(leftBlueStartPose)
                                            .lineTo(new Vector2d(35, 3))
                                            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                                slideTarget = sHigh;
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aStatic;
//                                                }
                                            })
                                            .lineToSplineHeading(new Pose2d(pickX1, pickY1, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX1, pickY1), () -> {
//                                               if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               bclaw.setPosition(0);
//                                               slideTarget = sLow;
                                            })
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               bclaw.setPosition(0);
//                                                slideTarget = sLow;
                                            })
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)

                                            // PARK
                                            .lineToSplineHeading(new Pose2d(35, 12.5, Math.toRadians(-90)))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                slideTarget = sLow;
                                            })
                                            .build()
                    );
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(b)
                    .start();
        } else if (park == 3) {
            b = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark())
                    .setConstraints(maxVel, maxAccel, 4.1715, Math.toRadians(60), trackWidth)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(leftBlueStartPose)
                                            .lineTo(new Vector2d(35, 3))
                                            .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                                slideTarget = sHigh;
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aStatic;
//                                                }
                                            })
                                            .lineToSplineHeading(new Pose2d(pickX1, pickY1, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX1, pickY1), () -> {
//                                               if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               bclaw.setPosition(0);
//                                               slideTarget = sLow;
                                            })
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               bclaw.setPosition(0);
//                                                slideTarget = sLow;
                                            })
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)
                                            .lineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(returnHead(dropHead, 1))))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                        armTarget = aPick;
//                                                    if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(clawClose);
//                                                        armTarget = armDrop;
//                                                    }
//                                                }
                                            })
                                            .waitSeconds(1)
                                            .addSpatialMarker(new Vector2d(dropX, dropY), () -> {
//                                                   if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                        claw.setPosition(1);
//                                                        armTarget = aStatic;
//                                                        if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) {
//                                                                slideTarget = sHigh;
//                                                        }
//                                                   }

                                            })
                                            .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead1, 1)) + 180))
                                            .addSpatialMarker(new Vector2d(pickX, pickY), () -> {
//                                               if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
//                                                   bclaw.setPosition(0.92);
//                                               }
                                            })
                                            .waitSeconds(1)

                                            // PARK
                                            .lineToConstantHeading(new Vector2d(35, 12))
                                            .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                                slideTarget = sLow;
                                            })
                                            .lineToConstantHeading(new Vector2d(35, 33))
                                            .splineToLinearHeading(new Pose2d(60, 35, Math.toRadians(-180)), Math.toRadians(0))
                                            .build()
                    );
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(b)
                    .start();
        }
    }
}