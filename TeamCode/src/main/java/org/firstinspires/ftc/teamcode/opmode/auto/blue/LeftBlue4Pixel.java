package org.firstinspires.ftc.teamcode.opmode.auto.blue;

import static org.firstinspires.ftc.teamcode.util.StartPoses.leftBlue;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
@Config
@Autonomous
public class LeftBlue4Pixel extends BaseOpMode {
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
                "russianblue.tflite", BLUELABEL);

        tensorflow.setMinConfidence(0.70);

        register(drop); // register so it runs the periodics in a loop while opmode is active

        intake.setDefaultCommand(new RunCommand(intake::stop, intake));

        // Drop ground pixel
        TrajectorySequence dropLeft = rrDrive.trajectorySequenceBuilder(leftBlue)
                .lineToSplineHeading(new Pose2d(29.5, 28, Math.toRadians(180)))
                .build();
        TrajectorySequence dropMiddle = rrDrive.trajectorySequenceBuilder(leftBlue)
                .lineToSplineHeading(new Pose2d(22, 27.5, Math.toRadians(200)))
                .build();
        TrajectorySequence dropRight = rrDrive.trajectorySequenceBuilder(leftBlue)
                .strafeRight(3)
                .lineToSplineHeading(new Pose2d(10.25, 32, Math.toRadians(180)))
                .lineTo(new Vector2d(6.5, 32))
                .build();

        // Drop to backdrop
        TrajectorySequence dropToBackdropLeft = rrDrive.trajectorySequenceBuilder(dropLeft.end())
                .lineToConstantHeading(new Vector2d(54, 41),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();
        TrajectorySequence dropToBackdropMiddle = rrDrive.trajectorySequenceBuilder(dropMiddle.end())
                .lineToSplineHeading(new Pose2d(54, 36, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(18))
                .build();
        TrajectorySequence dropToBackdropRight = rrDrive.trajectorySequenceBuilder(dropRight.end())
                .lineToConstantHeading(new Vector2d(54, 31),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

//        // Go Back
//        TrajectorySequence goBackLeft = rrDrive.trajectorySequenceBuilder(dropToBackdropLeft.end())
//                .forward(5,
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(18))
//                .build();
//        TrajectorySequence goBackMiddle = rrDrive.trajectorySequenceBuilder(dropToBackdropMiddle.end())
//                .forward(5,
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(18))
//                .build();
//        TrajectorySequence goBackRight = rrDrive.trajectorySequenceBuilder(dropToBackdropRight.end())
//                .forward(5,
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(18))
//                .build();

        // Localize
        TrajectorySequence localizeLeft = rrDrive.trajectorySequenceBuilder(dropToBackdropLeft.end())
                .lineToConstantHeading(new Vector2d(45,35),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(18))
                .build();
        TrajectorySequence localizeMiddle = rrDrive.trajectorySequenceBuilder(dropToBackdropMiddle.end())
                .lineToConstantHeading(new Vector2d(45,35),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(18))
                .build();
        TrajectorySequence localizeRight = rrDrive.trajectorySequenceBuilder(dropToBackdropRight.end())
                .lineToConstantHeading(new Vector2d(45,35),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(18))
                .build();

        // Cross Truss
        TrajectorySequence crossTrussLeft = rrDrive.trajectorySequenceBuilder(localizeLeft.end())
                .splineToConstantHeading(new Vector2d(18,35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .splineToConstantHeading(new Vector2d(-38,35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();
        TrajectorySequence crossTrussMiddle = rrDrive.trajectorySequenceBuilder(localizeMiddle.end())
                .splineToConstantHeading(new Vector2d(18,35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .splineToConstantHeading(new Vector2d(-38,35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();
        TrajectorySequence crossTrussRight = rrDrive.trajectorySequenceBuilder(localizeRight.end())
                .splineToConstantHeading(new Vector2d(18,35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .splineToConstantHeading(new Vector2d(-38,35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();

        // Go to Stack
        TrajectorySequence goToStackLeft = rrDrive.trajectorySequenceBuilder(crossTrussLeft.end())
                .lineToConstantHeading(new Vector2d(-63,36),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();
        TrajectorySequence goToStackMiddle = rrDrive.trajectorySequenceBuilder(crossTrussMiddle.end())
                .lineToConstantHeading(new Vector2d(-63,35.65),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();
        TrajectorySequence goToStackRight = rrDrive.trajectorySequenceBuilder(crossTrussRight.end())
                .lineToConstantHeading(new Vector2d(-63,36),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();

        // Move back
        TrajectorySequence moveBackLeft = rrDrive.trajectorySequenceBuilder(goToStackLeft.end())
                .back(1.5,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();
        TrajectorySequence moveBackMiddle = rrDrive.trajectorySequenceBuilder(goToStackMiddle.end())
                .back(1.5,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();
        TrajectorySequence moveBackRight = rrDrive.trajectorySequenceBuilder(goToStackRight.end())
                .back(1.5,
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        // Come to backdrop for whites
        TrajectorySequence crossForWhitesLeft = rrDrive.trajectorySequenceBuilder(moveBackLeft.end())
                .splineToConstantHeading(new Vector2d(45,35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();
        TrajectorySequence crossForWhitesMiddle = rrDrive.trajectorySequenceBuilder(moveBackMiddle.end())
                .splineToConstantHeading(new Vector2d(45,35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();
        TrajectorySequence crossForWhitesRight = rrDrive.trajectorySequenceBuilder(moveBackRight.end())
                .splineToConstantHeading(new Vector2d(45,35), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(23))
                .build();

        // Park
//        TrajectorySequence parkLeft = rrDrive.trajectorySequenceBuilder(goBackLeft.end())
//                .forward(5)
//                .lineToLinearHeading(new Pose2d(49.5, 59, Math.toRadians(180)),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .build();
//        TrajectorySequence parkMiddle = rrDrive.trajectorySequenceBuilder(goBackMiddle.end())
//                .forward(5)
//                .lineToLinearHeading(new Pose2d(49.5, 59, Math.toRadians(180)),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .build();
//        TrajectorySequence parkRight = rrDrive.trajectorySequenceBuilder(goBackRight.end())
//                .forward(5)
//                .lineToLinearHeading(new Pose2d(49.5, 59, Math.toRadians(180)),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .build();

        rrDrive.setPoseEstimate(leftBlue);

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
                                                    new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(750)), 3800).andThen(new InstantCommand(intake::stop, intake))
                                            )
                                    ));
                                    put(PropLocations.MIDDLE, new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                    new InstantCommand(tensorflow::shutdown, tensorflow),
                                                    new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropMiddle)),
                                                    new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                                    new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(500)), 3500).andThen(new InstantCommand(intake::stop, intake))
                                            )
                                    ));
                                    put(PropLocations.RIGHT, new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                    new InstantCommand(tensorflow::shutdown, tensorflow),
                                                    new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropRight)),
                                                    new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                                    new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(400)), 4400).andThen(new InstantCommand(intake::stop, intake))
                                            )
                                    ));
                                }},
                                () -> location
                        )
                ),
                new ParallelCommandGroup(
//                        new RunCommand(intake::push, intake).raceWith(new WaitCommand(350)).andThen(new InstantCommand(intake::stop, intake)),
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropLeft)));
                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropMiddle)));
                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropRight)));
                                }},
                                () -> location
                        )
//                        new DelayedCommand(new LiftSlideSmall(drop, intake),300)
                ),
                new WaitUntilCommand(() -> !rrDrive.isBusy()),
//                new WaitUntilCommand(() -> drop.getPosition() <= 655 && drop.getPosition() >= 640),
                new InstantCommand(drop::dropPixel, drop),
                new DelayedCommand(new RunCommand(intake::pushSlowAuto, intake).raceWith(new WaitCommand(800)), 450).andThen(new InstantCommand(intake::stop, intake)),
                new ParallelCommandGroup(
//                        new DelayedCommand(new DropSlide(drop), 100),
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(localizeLeft)));
                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(localizeMiddle)));
                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(localizeRight)));
                                }},
                                () -> location
                        )
                ),
                new WaitUntilCommand(() -> !rrDrive.isBusy()),
//                new InstantCommand(() -> localize = true), // start localizing
                new DelayedCommand(
                        new ParallelCommandGroup(
                                new SelectCommand(
                                        new HashMap<Object, Command>() {{
                                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(crossTrussLeft)));
                                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(crossTrussMiddle)));
                                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(crossTrussRight)));
                                        }},
                                        () -> location
                                )
                        ),550
                ),
                new WaitUntilCommand(() -> !rrDrive.isBusy()),
                new ParallelCommandGroup(
                        new InstantCommand(drop::setForFirstPixel, drop),
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(goToStackLeft)));
                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(goToStackMiddle)));
                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(goToStackRight)));
                                }},
                                () -> location
                        )
                ),
                new WaitUntilCommand(() -> !rrDrive.isBusy()),
                new RunCommand(intake::grab, intake).raceWith(new WaitCommand(750)).andThen(new InstantCommand(intake::stop, intake)),
                new ParallelCommandGroup(
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(moveBackLeft)));
                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(moveBackMiddle)));
                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(moveBackRight)));
                                }},
                                () -> location
                        ),
                        new InstantCommand(drop::setForSecondPixel, drop)
                ),
                new WaitUntilCommand(() -> !rrDrive.isBusy()),
                new RunCommand(intake::grab, intake).raceWith(new WaitCommand(400)).andThen(new InstantCommand(intake::stop, intake)),
//                new ParallelCommandGroup(
//                        new DelayedCommand(new SelectCommand(
//                                new HashMap<Object, Command>() {{
//                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(crossForWhitesLeft)));
//                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(crossForWhitesMiddle)));
//                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(crossForWhitesRight)));
//                                }},
//                                () -> location
//                        ), 100),
//                        new InstantCommand(drop::liftTray, drop)
//                )
                new InstantCommand(() -> aprilTagSubsystem.portal.stopStreaming()).andThen(new InstantCommand(aprilTagSubsystem::shutdown)) // todo: shutdown in parallel when nearing end of auto
        ));
    }

    @Override
    public void run() {
        super.run(); // since we are overriding in opmodes, this will actually run it
        rrDrive.update();
        telemetry.addData("Drive Pose", rrDrive.getPoseEstimate().toString());

        if (opModeIsActive() && !aprilTagSubsystem.getDetections().isEmpty()) {
            if (aprilTagSubsystem.getDetections().size() > 0) {
                AprilTagDetection currentDetection = aprilTagSubsystem.getDetections().get(0);

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