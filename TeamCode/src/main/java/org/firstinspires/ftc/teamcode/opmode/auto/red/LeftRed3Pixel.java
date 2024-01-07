package org.firstinspires.ftc.teamcode.opmode.auto.red;

import static org.firstinspires.ftc.teamcode.util.StartPoses.leftRed;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.outoftheboxrobotics.photoncore.Photon;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.commands.DropSlide;
import org.firstinspires.ftc.teamcode.commands.LiftSlideSmall;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.RRDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

import java.util.HashMap;
import java.util.List;

//@Photon
@Autonomous
public class LeftRed3Pixel extends BaseOpMode {
    private PropLocations location;
    private RRDriveSubsystem rrDrive;
    private List<Pose2d> tagPoses;
    private boolean noRRDrive = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        super.initialize();

        rrDrive = new RRDriveSubsystem(new SampleMecanumDrive(hardwareMap));
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "russianred.tflite", REDLABEL);

        tensorflow.setMinConfidence(0.70);

//        AprilTagSubsystem apriltagSubsystem = new AprilTagSubsystem(hardwareMap, "Webcam 1", "Webcam 2");
        register(drop, intake, bulkRead); // register so it runs the periodics in a loop while opmode is active
        intake.setDefaultCommand(new RunCommand(intake::stop, intake));

        // Drop ground pixel
        TrajectorySequence dropLeft = rrDrive.trajectorySequenceBuilder(leftRed)
                .lineToSplineHeading(new Pose2d(-34, -25.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();
        TrajectorySequence dropMiddle = rrDrive.trajectorySequenceBuilder(leftRed)
                .lineToSplineHeading(new Pose2d(-43, -25.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        TrajectorySequence dropRight = rrDrive.trajectorySequenceBuilder(leftRed)
                .lineToSplineHeading(new Pose2d(-43, -25.5, Math.toRadians(0)),
        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        // Go to stacks
        TrajectorySequence goToStacksLeft = rrDrive.trajectorySequenceBuilder(dropLeft.end())
                .lineToConstantHeading(new Vector2d(-48, -12),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        TrajectorySequence goToStacksMiddle = rrDrive.trajectorySequenceBuilder(dropMiddle.end())
                .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        TrajectorySequence goToStacksRight = rrDrive.trajectorySequenceBuilder(dropRight.end())
                .lineToLinearHeading(new Pose2d(-48, -12, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        // Cross Truss
        TrajectorySequence crossTrussLeft = rrDrive.trajectorySequenceBuilder(goToStacksLeft.end())
                .lineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        TrajectorySequence crossTrussMiddle = rrDrive.trajectorySequenceBuilder(goToStacksMiddle.end())
                .lineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        TrajectorySequence crossTrustRight = rrDrive.trajectorySequenceBuilder(goToStacksRight.end())
                .lineToSplineHeading(new Pose2d(35, -12, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        // Drop to backdrop
        TrajectorySequence dropToBackdropLeft = rrDrive.trajectorySequenceBuilder(crossTrussLeft.end())
                .lineToConstantHeading(new Vector2d(53, -31),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        TrajectorySequence dropToBackdropMiddle = rrDrive.trajectorySequenceBuilder(crossTrussMiddle.end())
                .lineToConstantHeading(new Vector2d(53, -37),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        TrajectorySequence dropToBackdropRight = rrDrive.trajectorySequenceBuilder(crossTrustRight.end())
                .lineToConstantHeading(new Vector2d(53, -43),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();

        // Park
        TrajectorySequence parkLeft = rrDrive.trajectorySequenceBuilder(dropToBackdropLeft.end())
                .lineToLinearHeading(new Pose2d(49, -15, Math.toRadians(180)))
                .build();
        TrajectorySequence parkMiddle = rrDrive.trajectorySequenceBuilder(dropToBackdropMiddle.end())
                .lineToLinearHeading(new Pose2d(49, -15, Math.toRadians(180)))
                .build();
        TrajectorySequence parkRight = rrDrive.trajectorySequenceBuilder(dropToBackdropRight.end())
                .lineToLinearHeading(new Pose2d(49, -15, Math.toRadians(180)))
                .build();

        rrDrive.setPoseEstimate(leftRed);

        // On init --> do nothing

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
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.LEFT, new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new InstantCommand(tensorflow::shutdown, tensorflow),
                                            new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropLeft)),
                                            new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                            new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(900)), 3600).andThen(new InstantCommand(intake::stop, intake))
                                    )
                            ));
                            put(PropLocations.MIDDLE, new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new InstantCommand(tensorflow::shutdown, tensorflow),
                                            new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropMiddle)),
                                            new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                            new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(900)), 3100).andThen(new InstantCommand(intake::stop, intake))
                                    )
                            ));
                            put(PropLocations.RIGHT, new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new InstantCommand(tensorflow::shutdown, tensorflow),
                                            new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropRight)),
                                            new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                            new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(900)), 3100).andThen(new InstantCommand(intake::stop, intake))
                                    )
                            ));
                        }},
                        () -> location
                )
//                new SelectCommand(
//                        new HashMap<Object, Command>() {{
//                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequence(goToStacksLeft)));
//                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequence(goToStacksMiddle)));
//                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequence(goToStacksRight)));
//                        }},
//                        () -> location
//                ),
//                new SelectCommand(
//                        new HashMap<Object, Command>() {{
//                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequence(crossTrussLeft)));
//                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequence(crossTrussMiddle)));
//                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequence(crossTrustRight)));
//                        }},
//                        () -> location
//                ),
//                new InstantCommand(() -> noRRDrive = true),
//                new LiftSlideSmall(drop),
//                new WaitUntilCommand(() -> drop.getPosition() <= 660 && drop.getPosition() >= 640),
//                new InstantCommand(drop::dropPixel, drop),
//                new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(1200)), 600).andThen(new InstantCommand(intake::stop, intake)),
//                new DelayedCommand(new DropSlide(drop), 200),
//                new WaitUntilCommand(() -> (drop.getPosition() <= 10 && drop.getPosition() <= -6)),
//                new InstantCommand(drop::liftTray),
//                new SelectCommand(
//                        new HashMap<Object, Command>() {{
//                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequence(parkLeft)));
//                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequence(parkMiddle)));
//                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequence(parkRight)));
//                        }},
//                        () -> location
//                )
        ));
    }

    @Override
    public void run() {
        super.run();
        rrDrive.update(noRRDrive);
        telemetry.update();
    }

    private enum PropLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }
}