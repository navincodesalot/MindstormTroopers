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
import com.outoftheboxrobotics.photoncore.Photon;
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
import org.firstinspires.ftc.teamcode.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RRDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;

@Photon
@Autonomous
public class RightRed2Pixel extends BaseOpMode {
    private PropLocations location;
    private RRDriveSubsystem rrDrive;
    private boolean noRRDrive = false;
    private double loopTime = 0.0;

    AprilTagSubsystem apriltagSubsystem = new AprilTagSubsystem(hardwareMap, "Webcam 1", "Webcam 2");

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        super.initialize();

        rrDrive = new RRDriveSubsystem(new SampleMecanumDrive(hardwareMap));
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "oldred.tflite", REDLABEL);

        tensorflow.setMinConfidence(0.69);

        apriltagSubsystem.switchCamera(2);

        register(drop); // register so it runs the periodics in a loop while opmode is active

        intake.setDefaultCommand(new RunCommand(intake::stop, intake));

        // Drop ground pixel
        TrajectorySequence dropLeft = rrDrive.trajectorySequenceBuilder(rightRed)
                .strafeLeft(2)
                .lineToSplineHeading(new Pose2d(8.5, -26.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(28))
                .build();
        TrajectorySequence dropMiddle = rrDrive.trajectorySequenceBuilder(rightRed)
                .lineToSplineHeading(new Pose2d(22, -25, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(28))
                .build();

        TrajectorySequence dropRight = rrDrive.trajectorySequenceBuilder(rightRed)
                .lineToSplineHeading(new Pose2d(31, -25, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(28))
                .build();

        // Drop to backdrop
        TrajectorySequence dropToBackdropLeft = rrDrive.trajectorySequenceBuilder(dropLeft.end())
                .lineToConstantHeading(new Vector2d(52.5, -31.5),
                        SampleMecanumDrive.getVelocityConstraint(26, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(26))
                .build();
        TrajectorySequence dropToBackdropMiddle = rrDrive.trajectorySequenceBuilder(dropMiddle.end())
                .lineToConstantHeading(new Vector2d(52.5, -37.5),
                        SampleMecanumDrive.getVelocityConstraint(26, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(26))
                .build();
        TrajectorySequence dropToBackdropRight = rrDrive.trajectorySequenceBuilder(dropRight.end())
                .lineToConstantHeading(new Vector2d(52.5, -45.5),
                        SampleMecanumDrive.getVelocityConstraint(26, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(26))
                .build();

        // Park
        TrajectorySequence parkLeft = rrDrive.trajectorySequenceBuilder(dropToBackdropLeft.end())
                .lineToLinearHeading(new Pose2d(49.5, -61, Math.toRadians(180)))
                .build();
        TrajectorySequence parkMiddle = rrDrive.trajectorySequenceBuilder(dropToBackdropMiddle.end())
                .lineToLinearHeading(new Pose2d(49.5, -61, Math.toRadians(180)))
                .build();
        TrajectorySequence parkRight = rrDrive.trajectorySequenceBuilder(dropToBackdropRight.end())
                .lineToLinearHeading(new Pose2d(49.5, -61, Math.toRadians(180)))
                .build();

        rrDrive.setPoseEstimate(rightRed);

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

        schedule(new SequentialCommandGroup(
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.LEFT, new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new InstantCommand(tensorflow::shutdown, tensorflow),
                                            new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropLeft)),
                                            new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                            new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(1500)), 3600).andThen(new InstantCommand(intake::stop, intake))
                                    )
                            ));
                            put(PropLocations.MIDDLE, new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new InstantCommand(tensorflow::shutdown, tensorflow),
                                            new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropMiddle)),
                                            new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                            new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(1500)), 3600).andThen(new InstantCommand(intake::stop, intake))
                                    )
                            ));
                            put(PropLocations.RIGHT, new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new InstantCommand(tensorflow::shutdown, tensorflow),
                                            new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropRight)),
                                            new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                            new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(1500)), 3600).andThen(new InstantCommand(intake::stop, intake))
                                    )
                            ));
                        }},
                        () -> location
                ),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequence(dropToBackdropLeft)));
                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequence(dropToBackdropMiddle)));
                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequence(dropToBackdropRight)));
                        }},
                        () -> location
                ),
                new InstantCommand(() -> noRRDrive = true),
                new LiftSlideSmall(drop),
                new WaitUntilCommand(() -> drop.getPosition() <= 670 && drop.getPosition() >= 635),
                new InstantCommand(drop::dropPixel, drop),
                new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(1200)), 600).andThen(new InstantCommand(intake::stop, intake)),
                new DelayedCommand(new DropSlide(drop), 200),
                new WaitUntilCommand(() -> (drop.getPosition() <= 14 && drop.getPosition() <= -6)),
                new DelayedCommand(new InstantCommand(drop::liftTray), 150),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequence(parkLeft)));
                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequence(parkMiddle)));
                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequence(parkRight)));
                        }},
                        () -> location
                ),
                new InstantCommand(apriltagSubsystem::shutdown) // todo: shutdown in parallel when nearing end of auto
        ));
    }

    @Override
    public void run() {
        super.run(); // since we are overriding in opmodes, this will actually run it
        rrDrive.update(noRRDrive);
        telemetry.addData("Drive Pose", rrDrive.getPoseEstimate().toString());

        AprilTagDetection currentDetection = apriltagSubsystem.getDetections().get(0);
        if (currentDetection.metadata != null) { // if a tag is detected
            double poseVelo = rrDrive.getPoseVelocity().vec().norm();

            if (poseVelo <= 0.25) { // and if robot velocity is <= 0.25 inches
                double heading = rrDrive.getPoseEstimate().getHeading();
                Vector2d localizedAprilTagVector = apriltagSubsystem.getFCPosition(currentDetection, heading);

                rrDrive.setPoseEstimate(localizedAprilTagVector.getX(), localizedAprilTagVector.getY(), heading);
                telemetry.addData("April Tag Pose", localizedAprilTagVector + ", " + heading);
            } else {
                telemetry.addData("April Tag Pose", "Robot velocity too high");
            }
        } else {
            telemetry.addData("April Tag Pose", "Tag not detected");
        }

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        bulkRead.read();
    }

    private enum PropLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }
}