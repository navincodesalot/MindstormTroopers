package org.firstinspires.ftc.teamcode.drive.code.auto.blue;

import static org.firstinspires.ftc.teamcode.drive.code.util.returns.returnHead;
import static org.firstinspires.ftc.teamcode.drive.code.util.returns.returnX;
import static org.firstinspires.ftc.teamcode.drive.code.util.returns.returnY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.code.util.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.code.util.detection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.code.util.detection.AprilTagsUtil;
import org.firstinspires.ftc.teamcode.drive.code.util.pidf.slidePIDF;
import org.firstinspires.ftc.teamcode.drive.code.util.startPoses;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class LeftBlue extends LinearOpMode {
    public AprilTagsUtil signalUtil = new AprilTagsUtil(hardwareMap, "Webcam 1", telemetry);;
    private DcMotorEx arm;
    private DcMotorEx slide;
    private Servo claw;
    private Servo bclaw;
    double slideTarget = 0;
    double sHigh = 2650;
    double sLow = 0;
    double aDrop = 20;
    double aPick = -100;
    double aStatic = -20;
    double armTarget = 0;
    double clawClose = 0.3;


    public void runOpMode() {
        PhotonCore.enable();
        double pickX = 42, pickY = 8, pickHead = -149;
        double dropX = 50, dropY = 12, dropHead = 0;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        bclaw = hardwareMap.get(Servo.class, "bclaw");

        Pose2d startPose = startPoses.leftBlueStartPose;

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        signalUtil.init();

        AprilTagDetectionPipeline.SignalPosition detection = signalUtil.getSignalPosition();
        drive.setPoseEstimate(startPose);

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(35, 8))
                .addTemporalMarker(2, () -> {
                    slideTarget = sHigh;
                })
                .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead, 1)) + 180))
                .addTemporalMarker(6, () -> {
                    bclaw.setPosition(0.92);
                })
                .waitSeconds(2.5)
                .addTemporalMarker(9, () -> {
                    bclaw.setPosition(0);  //bucket drop
                })
                .build();

        TrajectorySequence t2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(35, 8))
                .addTemporalMarker(2, () -> {
                    slideTarget = sHigh;
                })
                .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead, 1)) + 180))
                .addTemporalMarker(6, () -> {
                    bclaw.setPosition(0.92);
                })
                .waitSeconds(2.5)
                .addTemporalMarker(9, () -> {
                bclaw.setPosition(0); //bucket drop
                })
                .build();

        TrajectorySequence t3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(35, 8))
                .addTemporalMarker(2, () -> {
                    slideTarget = sHigh;
                })
                .lineToSplineHeading(new Pose2d(pickX, pickY, Math.toRadians(returnHead(pickHead, 1)) + 180))
                .addTemporalMarker(6, () -> {
                    bclaw.setPosition(0.92);
                })
                .waitSeconds(2.5)
                .addTemporalMarker(9, () -> {
                    bclaw.setPosition(0); //bucket drop
                })
                .build();

        TrajectorySequence p1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(35, 12, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(35, 35, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(60, 35, Math.toRadians(-90)))
                .addTemporalMarker(12, () -> {
                    slideTarget = sLow;
                })
                .build();
        TrajectorySequence p2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(35, 12.5, Math.toRadians(-90)))
                .addTemporalMarker(12, () -> {
                    slideTarget = sLow;
                })
                .build();

        TrajectorySequence p3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(12, 12, Math.toRadians(-90)))
                .addTemporalMarker(12, () -> {
                    slideTarget = sLow;
                })
                .build();

        waitForStart();
        if (detection == null || detection == AprilTagDetectionPipeline.SignalPosition.LEFT) {
            //run t1 traj
            drive.followTrajectorySequenceAsync(t1);
            drive.followTrajectorySequenceAsync(p1);
            PoseStorage.currentPose = drive.getPoseEstimate(); // Transfer the current pose to PoseStorage so we can use it in TeleOp
        } else if (detection == AprilTagDetectionPipeline.SignalPosition.MIDDLE) {
            //run t2 traj
            drive.followTrajectorySequenceAsync(t2);
            drive.followTrajectorySequenceAsync(p2);
            PoseStorage.currentPose = drive.getPoseEstimate();
        } else if (detection == AprilTagDetectionPipeline.SignalPosition.RIGHT) {
            //run t3 traj
            drive.followTrajectorySequenceAsync(t3);
            drive.followTrajectorySequenceAsync(p3);
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), slideTarget));
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));
        }
    }
}
