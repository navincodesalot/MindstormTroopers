package org.firstinspires.ftc.teamcode.opmode.auto.blue;

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
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
    private DcMotorEx i;
    private Servo leftS, rightS;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().enable();
        super.initialize();
        i = hardwareMap.get(DcMotorEx.class, "intake");
        leftS = hardwareMap.get(Servo.class, "leftServo");
        rightS = hardwareMap.get(Servo.class, "rightServo");
        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "blueprop.tflite", LABELS);

        tensorflow.setMinConfidence(0.75);
//        register(drop, intake);

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

            // On init
            drop.liftServo();
        }

        Pose2d leftBlueStartPos = new Pose2d(12, 66, Math.toRadians(270));
        rrDrive.setPoseEstimate(leftBlueStartPos);

        // Drop ground pixel (todo: wait 0.3 after)
        TrajectorySequence dropLeft = rrDrive.trajectorySequenceBuilder(leftBlueStartPos)
                .lineToSplineHeading(new Pose2d(12, 35, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence dropMiddle = rrDrive.trajectorySequenceBuilder(leftBlueStartPos) // what does reversed do
                .lineToSplineHeading(new Pose2d(12, 35, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence dropRight = rrDrive.trajectorySequenceBuilder(leftBlueStartPos) // what does reversed do
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

        waitForStart();

        if(location == PropLocations.MIDDLE) {
            rrDrive.followTrajectorySequence(dropMiddle);
            leftS.setPosition(0.5); // servos to ground
            rightS.setPosition(0.125);
            sleep(2000);

            i.setDirection(DcMotorEx.Direction.FORWARD);
            i.setPower(0.4);
            sleep(1500);
            i.setPower(0);

            sleep(1000);
            leftS.setPosition(0.455); //lift
            rightS.setPosition(0.085);

            rrDrive.followTrajectorySequence(parkMiddle);
        } else if (location == PropLocations.RIGHT) {
            rrDrive.followTrajectorySequence(dropRight);
            leftS.setPosition(0.5); // servos to ground
            rightS.setPosition(0.125);
            sleep(2000);

            i.setDirection(DcMotorEx.Direction.FORWARD);
            i.setPower(0.4);
            sleep(1500);
            i.setPower(0);

            sleep(1000);
            leftS.setPosition(0.455); //lift
            rightS.setPosition(0.085);
            rrDrive.followTrajectorySequence(parkRight);
        } else {
            rrDrive.followTrajectorySequence(dropLeft);
            leftS.setPosition(0.5); // servos to ground
            rightS.setPosition(0.125);
            sleep(2000);

            i.setDirection(DcMotorEx.Direction.FORWARD);
            i.setPower(0.4);
            sleep(1500);
            i.setPower(0);

            sleep(1000);
            leftS.setPosition(0.455); //lift
            rightS.setPosition(0.085);
            rrDrive.followTrajectorySequence(parkLeft);
        }
        PoseStorage.currentPose = rrDrive.getPoseEstimate(); //send pose to tele
    }

    @Override
    public void run() {
        super.run();
        drop.periodic(); //todo do we need this for slides?
        rrDrive.update(); // since we are running some async, we gotta update while opmode is active
    }

    private enum PropLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }
}