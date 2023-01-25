package org.firstinspires.ftc.teamcode.drive.code.auto.red;
import static org.firstinspires.ftc.teamcode.drive.code.util.returns.returnHead;
import static org.firstinspires.ftc.teamcode.drive.code.util.returns.returnX;
import static org.firstinspires.ftc.teamcode.drive.code.util.returns.returnY;
import static org.firstinspires.ftc.teamcode.drive.code.util.startPoses.pickHead;
import static org.firstinspires.ftc.teamcode.drive.code.util.startPoses.pickX;
import static org.firstinspires.ftc.teamcode.drive.code.util.startPoses.pickY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.code.util.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.code.util.detection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.code.util.detection.AprilTagsUtil;
import org.firstinspires.ftc.teamcode.drive.code.util.pidf.armPIDF;
import org.firstinspires.ftc.teamcode.drive.code.util.pidf.slidePIDF;
import org.firstinspires.ftc.teamcode.drive.code.util.startPoses;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class LeftRed extends LinearOpMode {
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        bclaw = hardwareMap.get(Servo.class, "bclaw");

        Pose2d startPose = startPoses.leftRedStartPose;

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        signalUtil.init();

        AprilTagDetectionPipeline.SignalPosition detection = signalUtil.getSignalPosition();
        drive.setPoseEstimate(startPose);

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(35, returnY(3)))
                .lineTo(new Vector2d(35, returnY(8)))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideTarget = sHigh;
                })
                .lineToSplineHeading(new Pose2d(pickX, returnY(pickY), Math.toRadians(returnHead(pickHead))))
                .addSpatialMarker(new Vector2d(pickX, returnY(pickY)), () -> {
                    if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) {
                        bclaw.setPosition(0.92);
                    }
                })
                .waitSeconds(2.5)
                .addSpatialMarker(new Vector2d(pickX, returnY(pickY)), () -> {
                        bclaw.setPosition(0);
                })
                .build();

        TrajectorySequence p1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(12, returnY(12), Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideTarget = sLow;
                })
                .build();
        TrajectorySequence p2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(35, returnY(12.5), Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideTarget = sLow;
                })
                .build();

        TrajectorySequence p3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(35, returnY(12), Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(35, returnY(35), Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(60, returnY(35), Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideTarget = sLow;
                })
                .build();

        waitForStart();
        if (detection == null || detection == AprilTagDetectionPipeline.SignalPosition.LEFT) {
            //run t1 traj
            drive.followTrajectorySequenceAsync(cycle);
            drive.followTrajectorySequenceAsync(p1);
            PoseStorage.currentPose = drive.getPoseEstimate(); // Transfer the current pose to PoseStorage so we can use it in TeleOp
            signalUtil.stopCamera();
        } else if (detection == AprilTagDetectionPipeline.SignalPosition.MIDDLE) {
            //run t2 traj
            drive.followTrajectorySequenceAsync(cycle);
            drive.followTrajectorySequenceAsync(p2);
            PoseStorage.currentPose = drive.getPoseEstimate();
            signalUtil.stopCamera();
        } else if (detection == AprilTagDetectionPipeline.SignalPosition.RIGHT) {
            //run t3 traj
            drive.followTrajectorySequenceAsync(cycle);
            drive.followTrajectorySequenceAsync(p3);
            PoseStorage.currentPose = drive.getPoseEstimate();
            signalUtil.stopCamera();
        }
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), slideTarget));
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));
        }
    }
}
