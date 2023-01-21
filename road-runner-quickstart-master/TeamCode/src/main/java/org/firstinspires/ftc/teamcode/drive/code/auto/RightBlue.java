/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.code.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.code.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.code.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.code.pidf.slidePIDF;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Autonomous

public class RightBlue extends LinearOpMode {
    private DcMotorEx arm;
//    private DcMotorEx slide;
    private Servo claw;
    private Servo bclaw;
    OpenCvCamera camera;
    double target = 0;
    double high = 2650;
    double low = 0;
    double clawClose = 0.3;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 2, 10, 18 from the 36h11 family
    int LEFT = 2;
    int MIDDLE = 10;
    int RIGHT = 18;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        PhotonCore.enable();
        double pickX = 42, pickY = 8, pickHead = -149;
        double dropX = 50, dropY = 12, dropHead = 0;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        slide = hardwareMap.get(DcMotorEx.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        bclaw = hardwareMap.get(Servo.class, "bclaw");

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

//        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        slide.setDirection(DcMotorSimple.Direction.REVERSE);



        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: 10, y: 15, and facing 90 degrees (turned counter-clockwise)

        Pose2d leftBlueStartPose = new Pose2d(35, 61, Math.toRadians(-90));
        Pose2d rightBlueStartPose = new Pose2d(returnX(35), 61, Math.toRadians(-90));

        Pose2d leftRedStartPose = new Pose2d(35, returnY(61), Math.toRadians(-270));
        Pose2d rightRedStartPose = new Pose2d(returnX(35), returnY(61), Math.toRadians(-270));

        drive.setPoseEstimate(rightBlueStartPose);

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(rightBlueStartPose) // increment y to go further towards blue wall
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(returnX(35), 8))
//                .addTemporalMarker(2, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), high));
//                })
                .lineToSplineHeading(new Pose2d(returnX(pickX), pickY, Math.toRadians(returnHead(pickHead, 1))))
//                .addTemporalMarker(6, () -> {
//                    bclaw.setPosition(0.92);
//                })
                .waitSeconds(2.5) //bucket drop
//                .addTemporalMarker(9, () -> {
//                    bclaw.setPosition(0);
//                })
                .lineToSplineHeading(new Pose2d(returnX(12), 12, Math.toRadians(-90)))
//                    .addTemporalMarker(12, () -> {
//                        slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), low));
//                    })
                .build();

        TrajectorySequence t2 = drive.trajectorySequenceBuilder(rightBlueStartPose) // increment y to go further towards blue wall
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(returnX(35), 8))
//                .addTemporalMarker(2, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), high));
//                })
                .lineToSplineHeading(new Pose2d(returnX(pickX), pickY, Math.toRadians(returnHead(pickHead, 1))))
//                .addTemporalMarker(6, () -> {
//                bclaw.setPosition(0.92);
//                })
                .waitSeconds(2.5) //bucket drop
//                .addTemporalMarker(9, () -> {
//                bclaw.setPosition(0);
//                })
                .lineToSplineHeading(new Pose2d(returnX(35), 12.5, Math.toRadians(-90)))
//                .addTemporalMarker(12, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), low));
//                })
                .build();

        TrajectorySequence t3 = drive.trajectorySequenceBuilder(rightBlueStartPose) // increment y to go further towards blue wall
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(returnX(35), 8))
//                .addTemporalMarker(2, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), high));
//                })
                .lineToSplineHeading(new Pose2d(returnX(pickX), pickY, Math.toRadians(returnHead(pickHead, 1))))
//                .addTemporalMarker(6, () -> {
//                    bclaw.setPosition(0.92);
//                })
                .waitSeconds(2.5) //bucket drop
//                .addTemporalMarker(9, () -> {
//                    bclaw.setPosition(0);
//                })
                .lineToSplineHeading(new Pose2d(returnX(35), 12, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(returnX(35), 35, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(returnX(60), 35, Math.toRadians(-90)))
//                .addTemporalMarker(12, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), low));
//                })
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            arm.setTargetPosition(0);
//            slide.setTargetPosition(0);
            claw.setPosition(clawClose);
            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else {
                telemetry.addLine("Don't see tag of interest :(");
                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actual autonomous*/
        // need to import rr and trajectories todo

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            //trajectory
            drive.followTrajectorySequence(t1);
            PoseStorage.currentPose = drive.getPoseEstimate(); // Transfer the current pose to PoseStorage so we can use it in TeleOp
        } else if (tagOfInterest.id == MIDDLE) {
            //trajectory
            drive.followTrajectorySequence(t2);
            PoseStorage.currentPose = drive.getPoseEstimate(); // Transfer the current pose to PoseStorage so we can use it in TeleOp
        } else if (tagOfInterest.id == RIGHT) {
            //trajectory
            drive.followTrajectorySequence(t3);
            PoseStorage.currentPose = drive.getPoseEstimate(); // Transfer the current pose to PoseStorage so we can use it in TeleOp
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //     while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public static double returnX(double x) {
        return x * (-1);
    }
    public static double returnHead(double h) { h = Math.abs(h); return h -= 180; }
    public static double returnHead(double h, int i) { h = Math.abs(h); return h -= 360; }
    public static double returnHead(double h, String s) { h = Math.abs(h); return h -= 180; }
    public static double returnY(double y) { return y * (-1); }
}