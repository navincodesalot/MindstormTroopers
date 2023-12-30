package org.firstinspires.ftc.teamcode.opmode.auto.red;

import static org.firstinspires.ftc.teamcode.util.StartPoses.rightRed;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import java.util.HashMap;

@Autonomous
public class RightRed extends BaseOpMode {
    private PropLocations location;
    private final int servoTime = 400;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        super.initialize();
        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "russianred.tflite", REDLABEL);

        tensorflow.setMinConfidence(0.70);
        register(drop, intake, bulkRead); // register so it runs the periodics in a loop while opmode is active

        // Drop ground pixel
        TrajectorySequence dropLeft = rrDrive.trajectorySequenceBuilder(rightRed)
                .lineToSplineHeading(new Pose2d(35, -29, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence dropMiddle = rrDrive.trajectorySequenceBuilder(rightRed)
                .lineToSplineHeading(new Pose2d(12, -35, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence dropRight = rrDrive.trajectorySequenceBuilder(rightRed)
                .lineToSplineHeading(new Pose2d(35, -29, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(37, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
//
//        // Move away (not needed for left)
//        TrajectorySequence moveAwayMiddle = rrDrive.trajectorySequenceBuilder(dropMiddle.end())
//                .lineTo(new Vector2d(12, -40))
//                .build();
//        TrajectorySequence moveAwayRight = rrDrive.trajectorySequenceBuilder(dropRight.end())
//                .back(2)
//                .build();

        // Drop to backdrop
        TrajectorySequence dropToBackdropLeft = rrDrive.trajectorySequenceBuilder(dropLeft.end())
                .lineTo(new Vector2d(45, -29))
                .build();
        TrajectorySequence dropToBackdropMiddle = rrDrive.trajectorySequenceBuilder(dropMiddle.end())
                .lineToSplineHeading(new Pose2d(45, -37, Math.toRadians(180)))
                .build();
        TrajectorySequence dropToBackdropRight = rrDrive.trajectorySequenceBuilder(dropRight.end())
                .lineToConstantHeading(new Vector2d(47, -44))
                .build();

        // Park
        TrajectorySequence parkLeft = rrDrive.trajectorySequenceBuilder(dropToBackdropLeft.end())
                .forward(6)
                .strafeLeft(10)
                .splineToLinearHeading(new Pose2d(59, -60, Math.toRadians(180)), Math.toRadians(0))
                .build();
        TrajectorySequence parkMiddle = rrDrive.trajectorySequenceBuilder(dropToBackdropMiddle.end())
                .forward(4)
                .strafeLeft(4)
                .splineToLinearHeading(new Pose2d(59, -60, Math.toRadians(180)), Math.toRadians(0))
                .build();
        TrajectorySequence parkRight = rrDrive.trajectorySequenceBuilder(dropToBackdropRight.end())
                .forward(5)
                .strafeLeft(3)
                .splineToConstantHeading(new Vector2d(59, -60), Math.toRadians(0))
                .build();

        rrDrive.setPoseEstimate(rightRed);

        // On init
        drop.liftHighTray();

        while (opModeInInit()) {
            Recognition bestDetection = tensorflow.getBestDetection();
            location = PropLocations.LEFT;

            if (bestDetection != null) {
                double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2;
                if (x > 365) {
                    location = PropLocations.RIGHT;
                } else if (x > 160 && x < 365) {
                    location = PropLocations.MIDDLE;
                }
            }

//            telemetry.addData("FPS", tensorflow.portal.getFps()); // remove tele except loco
            telemetry.addData("Current Location", location.toString());
//            telemetry.addData("Confidence", String.format("%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();

        }

        // Todo make sure to use async trajs when running in parallel
        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(tensorflow::shutdown),
                        new DelayedCommand(new InstantCommand(drop::pickupPixel), 1000),
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropLeft)));
                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropMiddle)));
                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropRight)));
                                }},
                                () -> location
                        )
                )

//                // drop ground pixel
//                new InstantCommand(drop::pickupPixel, drop),
//                new WaitCommand(servoTime), // <- 400ms
//                new RunCommand(() -> intake.pushSlow(0.6), intake).raceWith(new WaitCommand(1000)).andThen(new RunCommand(intake::stop, intake)),
//
//
//                new ParallelCommandGroup( // todo: these are async trajs (drive while lifting slides)
//                        new InstantCommand(drop::slideLift, drop),
//                        // this will run the traj around 500ms after so the slides can go up and the servos don't drag on the field while running the trajectory
//                        new DelayedCommand(new SelectCommand(
//                                new HashMap<Object, Command>() {{
//                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropLeft)));
//                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropMiddle)));
//                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropRight)));
//                                }},
//                                () -> location
//                            ), 500)
//                ),
//
//                // drop pixel
//                new ParallelCommandGroup( // todo: watch syntax here
//                        new InstantCommand(() -> {
//                            drop.dropPixel();
//                        }, drop)
//                ),
//                new WaitCommand(1000),
//
//                //todo fix logic: here i need to reset servos, bring slides down, then lift servos down WHILE driving rr path to save time (make sure to not flip)
//                new ParallelCommandGroup( // example: https://github.com/Watt-sUP/CenterStage2023/blob/e2e4e643bfca7fcf61e531f743066df861e16ee3/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/BlueLong.java#L150
//                        new InstantCommand(drop::pickupPixel, drop),
//                        new DelayedCommand(new InstantCommand(drop::slideIdle, drop), servoTime), // offsetted by the servo time
//                        new SequentialCommandGroup(
//                                new WaitUntilCommand(() -> drop.getPosition() <= 700), // wait until slides are low enough before starting traj
//                                new SelectCommand(
//                                        new HashMap<Object, Command>() {{
//                                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(parkLeft)));
//                                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(parkMiddle)));
//                                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(parkRight)));
//                                        }},
//                                        () -> location
//                                )
//                        )
//                )
        ));

        PoseStorage.currentPose = rrDrive.getPoseEstimate(); //send pose to tele
    }

    @Override
    public void run() {
        super.run();
        rrDrive.update(); // since we are running some async, we gotta update while opmode is active
    }

    private enum PropLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }
}