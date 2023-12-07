package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.TensorflowSubsystem;

@Autonomous
public class RightRed extends BaseOpMode {
    private PropLocations location;

    @Override
    public void initialize() {
        super.initialize();
        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "redprop.tflite", LABELS);

        //visionPortal.stopLiveView(); // todo: when in comp
        tensorflow.setMinConfidence(0.75);

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

            telemetry.addData("FPS", tensorflow.portal.getFps()); //todo remove tele except loco
            telemetry.addData("Current Location", location.toString());
            telemetry.addData("Confidence", String.format("%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();
        }

//        imu.reset(); todo
        // TODO: trajs go here
        drop.liftServo();

        Pose2d rightRedStartPos = new Pose2d(12, -70, Math.toRadians(90));
        rrDrive.setPoseEstimate(rightRedStartPos);

        Trajectory dropMiddle = rrDrive.trajectoryBuilder(rightRedStartPos, true)
                .lineToSplineHeading(new Pose2d(12, -35.5, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
// todo: strafe 1 inch over before park before spline curve
                .build();

//        drop.liftServo();

        waitForStart();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            new InstantCommand(tensorflow::shutdown),
            new InstantCommand(() -> rrDrive.followTrajectory(dropMiddle))
        ));

    }
    private enum PropLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }
}
