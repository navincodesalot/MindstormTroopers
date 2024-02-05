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
import org.firstinspires.ftc.teamcode.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RRDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;

//@Photon
//@Config
@Autonomous
public class RightRed2Pixel extends BaseOpMode {
    private PropLocations location;
    private RRDriveSubsystem rrDrive;
    private double loopTime = 0.0;
    private AprilTagSubsystem aprilTagSubsystem;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        rrDrive = new RRDriveSubsystem(new SampleMecanumDrive(hardwareMap));
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        super.initialize();

        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "romanianred.tflite", REDLABEL);

        tensorflow.setMinConfidence(0.70);

        register(drop); // register so it runs the periodics in a loop while opmode is active

        intake.setDefaultCommand(new RunCommand(intake::stop, intake));

        // Drop ground pixel
        TrajectorySequence dropLeft = rrDrive.trajectorySequenceBuilder(rightRed)
                .back(3)
                .strafeLeft(7)
                .lineToSplineHeading(new Pose2d(14, -47, Math.toRadians(-180)))
                .lineToConstantHeading(new Vector2d(8.5, -34))
                .build();
        TrajectorySequence dropMiddle = rrDrive.trajectorySequenceBuilder(rightRed)
                .lineToSplineHeading(new Pose2d(22, -24.5, Math.toRadians(-200)))
                .build();
        TrajectorySequence dropRight = rrDrive.trajectorySequenceBuilder(rightRed)
                .lineToSplineHeading(new Pose2d(25.5, -18.5, Math.toRadians(-112)))
                .build();

        // Drop to backdrop
        TrajectorySequence dropToBackdropLeft = rrDrive.trajectorySequenceBuilder(dropLeft.end())
                .lineToConstantHeading(new Vector2d(56, -30),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();
        TrajectorySequence dropToBackdropMiddle = rrDrive.trajectorySequenceBuilder(dropMiddle.end())
                .lineToSplineHeading(new Pose2d(56, -36, Math.toRadians(-180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();
        TrajectorySequence dropToBackdropRight = rrDrive.trajectorySequenceBuilder(dropRight.end())
                .back(3)
                .lineToSplineHeading(new Pose2d(56, -45, Math.toRadians(-180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();

        // Go Back
        TrajectorySequence goBackLeft = rrDrive.trajectorySequenceBuilder(dropToBackdropLeft.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(18))
                .build();
        TrajectorySequence goBackMiddle = rrDrive.trajectorySequenceBuilder(dropToBackdropMiddle.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(18))
                .build();
        TrajectorySequence goBackRight = rrDrive.trajectorySequenceBuilder(dropToBackdropRight.end())
                .forward(5,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(18))
                .build();
        // Park
        TrajectorySequence parkLeft = rrDrive.trajectorySequenceBuilder(goBackLeft.end())
                .forward(5)
                .lineToLinearHeading(new Pose2d(49.5, -60, Math.toRadians(-180)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        TrajectorySequence parkMiddle = rrDrive.trajectorySequenceBuilder(goBackMiddle.end())
                .forward(5)
                .lineToLinearHeading(new Pose2d(49.5, -60, Math.toRadians(-180)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        TrajectorySequence parkRight = rrDrive.trajectorySequenceBuilder(goBackRight.end())
                .forward(5)
                .lineToLinearHeading(new Pose2d(49.5, -60, Math.toRadians(-180)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
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
                new ParallelCommandGroup(
                        new InstantCommand(() -> aprilTagSubsystem = new AprilTagSubsystem(hardwareMap)),
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                    new InstantCommand(tensorflow::shutdown, tensorflow),
                                                    new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropLeft)),
                                                    new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                                    new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(750)), 3400).andThen(new InstantCommand(intake::stop, intake))
                                            )
                                    ));
                                    put(PropLocations.MIDDLE, new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                    new InstantCommand(tensorflow::shutdown, tensorflow),
                                                    new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropMiddle)),
                                                    new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                                    new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(550)), 3300).andThen(new InstantCommand(intake::stop, intake))
                                            )
                                    ));
                                    put(PropLocations.RIGHT, new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                    new InstantCommand(tensorflow::shutdown, tensorflow),
                                                    new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropRight)),
                                                    new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                                    new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(600)), 5000).andThen(new InstantCommand(intake::stop, intake))
                                            )
                                    ));
                                }},
                                () -> location
                        )
                ),
                new ParallelCommandGroup(
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropLeft)));
                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropMiddle)));
                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropRight)));
                                }},
                                () -> location
                        ),
                        new DelayedCommand(new LiftSlideSmall(drop, intake),600)
                ),
                new WaitUntilCommand(() -> !rrDrive.isBusy()),
                new WaitUntilCommand(() -> drop.getPosition() <= 635 && drop.getPosition() >= 605),
                new InstantCommand(drop::dropPixel, drop),
                new DelayedCommand(new RunCommand(intake::pushSlowAuto, intake).raceWith(new WaitCommand(700)), 450).andThen(new InstantCommand(intake::stop, intake)),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(goBackLeft)));
                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(goBackMiddle)));
                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(goBackRight)));
                        }},
                        () -> location
                ),
                new WaitUntilCommand(() -> !rrDrive.isBusy()),
                new DelayedCommand(new DropSlide(drop), 250),
                new WaitUntilCommand(() -> (drop.getPosition() <= 20 && drop.getPosition() >= -10)),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(parkLeft)));
                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(parkMiddle)));
                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(parkRight)));

                        }},
                        () -> location
                ),
                new WaitUntilCommand(() -> !rrDrive.isBusy())
                // new InstantCommand(aprilTagSubsystem::shutdown) // todo: shutdown in parallel when nearing end of auto
        ));
    }

    @Override
    public void run() {
        super.run(); // since we are overriding in opmodes, this will actually run it
        rrDrive.update();
//        telemetry.addData("Drive Pose", rrDrive.getPoseEstimate().toString());

//        List<AprilTagDetection> detected = aprilTagSubsystem.getDetections();

        if (opModeIsActive() && !aprilTagSubsystem.getDetections().isEmpty()) {
            if (aprilTagSubsystem.getDetections().size() > 0) {
                AprilTagDetection currentDetection = aprilTagSubsystem.getDetections().get(0); // todo: fix later

                if (currentDetection.metadata != null) { // if a tag is detected
                    double poseVelo = Math.abs(rrDrive.getPoseVelocity().vec().norm());
                    Pose2d currentPose = rrDrive.getPoseEstimate();

                    if (poseVelo <= 0.25) { // if robot velocity is <= 0.25 inches
                        Vector2d localizedAprilTagVector = aprilTagSubsystem.getFCPosition(currentDetection, currentPose.getHeading(), "BLUE");

                        rrDrive.setPoseEstimate(new Pose2d(localizedAprilTagVector.getX(), localizedAprilTagVector.getY(), currentPose.getHeading()));

                        telemetry.addData("updated drive pose", rrDrive.getPoseEstimate().toString());
                        telemetry.addData("April Tag Pose", localizedAprilTagVector + ", " + Math.toDegrees(currentPose.getHeading()));
                    } else {
                        telemetry.addData("April Tag Pose", "Robot velocity too high / boolean false");
                    }
                }
            } else {
                telemetry.addData("April Tag Pose", "Tag not detected");
            }
        }

//        telemetry.addData("slide pos", drop.getPosition());
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