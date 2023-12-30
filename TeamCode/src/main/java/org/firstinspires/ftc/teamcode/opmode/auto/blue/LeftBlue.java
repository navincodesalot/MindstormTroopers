package org.firstinspires.ftc.teamcode.opmode.auto.blue;

import static org.firstinspires.ftc.teamcode.util.StartPoses.leftBlue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.HashMap;

@Autonomous
public class LeftBlue extends BaseOpMode {
    private PropLocations location;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        super.initialize();
        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "russianblue.tflite", BLUELABEL);

        tensorflow.setMinConfidence(0.75);
        register(drop, intake, bulkRead); // register so it runs the periodics

        // Drop ground pixel
        TrajectorySequence dropLeft = rrDrive.trajectorySequenceBuilder(leftBlue)
                .lineToSplineHeading(new Pose2d(12, 35, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence dropMiddle = rrDrive.trajectorySequenceBuilder(leftBlue) // what does reversed do
                .lineToSplineHeading(new Pose2d(12, 35, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence dropRight = rrDrive.trajectorySequenceBuilder(leftBlue) // what does reversed do
                .lineToSplineHeading(new Pose2d(12, 35, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Park
        TrajectorySequence parkLeft = rrDrive.trajectorySequenceBuilder(dropLeft.end())
                .lineToSplineHeading(new Pose2d(12, 66, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(22, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(59, 60, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence parkMiddle = rrDrive.trajectorySequenceBuilder(dropMiddle.end())
                .lineToSplineHeading(new Pose2d(12, 66, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(22, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(59, 60, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(22, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence parkRight = rrDrive.trajectorySequenceBuilder(dropRight.end())
                .lineToSplineHeading(new Pose2d(12, 66, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(22, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(59, 60, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(22, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        rrDrive.setPoseEstimate(leftBlue);

        // On init
        drop.liftTray();

        while (opModeInInit()) {
            Recognition bestDetection = tensorflow.getBestDetection();
            location = PropLocations.MIDDLE;

            if (bestDetection != null) {
                double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2;
                if (x < 150) {
                    location = PropLocations.LEFT;
                } else if (x > 150 && x < 440) {
                    location = PropLocations.MIDDLE;
                } else {
                    location = PropLocations.RIGHT;
                }
            }

//            telemetry.addData("FPS", tensorflow.portal.getFps()); // remove tele except loco
            telemetry.addData("Current Location", location.toString());
//            telemetry.addData("Confidence", String.format("%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();
        }

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(tensorflow::shutdown),
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequence(dropLeft)));
                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequence(dropMiddle)));
                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequence(dropRight)));
                                }},
                                () -> location
                        )
                ),

                // drop ground pixel
                new InstantCommand(drop::pickupPixel),
                new WaitCommand(400),
                new RunCommand(() -> intake.pushSlow(0.6), intake).raceWith(new WaitCommand(1000)).andThen(new RunCommand(intake::stop, intake)),

                //todo: maybe do in parallel? (reset for lift)
                new InstantCommand(drop::liftTray),
                new WaitCommand(250),

                new ParallelCommandGroup(
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequence(dropLeft)));
                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequence(dropMiddle)));
                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequence(dropRight)));
                                }},
                                () -> location
                        ),
                        new InstantCommand(drop::slideMiddle)
                )

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