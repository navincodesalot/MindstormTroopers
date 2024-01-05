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
//import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.HashMap;
import java.util.List;

//@Photon
@Autonomous
public class RightRed2Pixel extends BaseOpMode {
    private PropLocations location;
    private final int servoTime = 400;
    private List<Pose2d> tagPoses;
    protected SampleMecanumDrive rrDrive;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        super.initialize();

        rrDrive = new SampleMecanumDrive(hardwareMap);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "russianred.tflite", REDLABEL);

        tensorflow.setMinConfidence(0.70);

        AprilTagSubsystem apriltagSubsystem = new AprilTagSubsystem(hardwareMap, "Webcam 1", "Webcam 2");
        register(drop, intake, bulkRead); // register so it runs the periodics in a loop while opmode is active
        intake.setDefaultCommand(new RunCommand(intake::stop, intake));

        // Drop ground pixel
        TrajectorySequence dropLeft = rrDrive.trajectorySequenceBuilder(rightRed)
                .strafeLeft(2)
                .lineToSplineHeading(new Pose2d(10.5, -26.5, Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence dropMiddle = rrDrive.trajectorySequenceBuilder(rightRed)
                .lineToSplineHeading(new Pose2d(23, -23, Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence dropRight = rrDrive.trajectorySequenceBuilder(rightRed)
                .lineToSplineHeading(new Pose2d(31, -22.75, Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Drop to backdrop
        TrajectorySequence dropToBackdropLeft = rrDrive.trajectorySequenceBuilder(dropLeft.end())
                .lineToConstantHeading(new Vector2d(47, -29))
                .build();
        TrajectorySequence dropToBackdropMiddle = rrDrive.trajectorySequenceBuilder(dropMiddle.end())
                .lineToConstantHeading(new Vector2d(47, -35))
                .build();
        TrajectorySequence dropToBackdropRight = rrDrive.trajectorySequenceBuilder(dropRight.end())
                .lineToConstantHeading(new Vector2d(47, -45))
                .build();

        // Park
        TrajectorySequence parkLeft = rrDrive.trajectorySequenceBuilder(dropToBackdropLeft.end())
                .lineToLinearHeading(new Pose2d(47, -60, Math.toRadians(180)))
                .build();
        TrajectorySequence parkMiddle = rrDrive.trajectorySequenceBuilder(dropToBackdropMiddle.end())
                .lineToLinearHeading(new Pose2d(47, -60, Math.toRadians(180)))
                .build();
        TrajectorySequence parkRight = rrDrive.trajectorySequenceBuilder(dropToBackdropRight.end())
                .lineToLinearHeading(new Pose2d(47, -60, Math.toRadians(180)))
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

        // Todo make sure to use async trajs when running in parallel
        schedule(new SequentialCommandGroup(
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.LEFT, new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new InstantCommand(tensorflow::shutdown, tensorflow),
                                            new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropLeft)),
                                            new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                            new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(1000)), 3000)
                                    )
                            ));
                            put(PropLocations.MIDDLE, new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new InstantCommand(tensorflow::shutdown, tensorflow),
                                            new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropMiddle)),
                                            new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
                                            new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(1000)), 3000)
                                    )
                            ));
                            put(PropLocations.RIGHT, new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new InstantCommand(tensorflow::shutdown, tensorflow),
                                            new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropRight)),
                                            new DelayedCommand(new InstantCommand(drop::pickupPixel, drop), 1000),
//                                            new WaitUntilCommand(() -> (rrDrive.getPoseEstimate().getY() <= (-25.0)) && (rrDrive.getPoseEstimate().getY() >= (-23.0))), todo
                                            new DelayedCommand(new RunCommand(intake::push, intake).raceWith(new WaitCommand(1000)), 3000)
                                    )
                            ));
                        }},
                        () -> location
                ),
                new ParallelCommandGroup(
                        new InstantCommand(drop::liftTray, drop),
                        new SelectCommand(
                                new HashMap<Object, Command>() {{
                                    put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropLeft)));
                                    put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropMiddle)));
                                    put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(dropToBackdropRight)));
                                }},
                                () -> location
                        )
                ),
                // todo: when lifting slide
//                new LiftSlide(drop),
//                new WaitUntilCommand(() -> (drop.getPosition() <= 1080) && (drop.getPosition() >= 1060)),

                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(PropLocations.LEFT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(parkLeft)));
                            put(PropLocations.MIDDLE, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(parkMiddle)));
                            put(PropLocations.RIGHT, new InstantCommand(() -> rrDrive.followTrajectorySequenceAsync(parkRight)));
                        }},
                        () -> location
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