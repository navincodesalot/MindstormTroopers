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

package org.firstinspires.ftc.teamcode.drive.code.auto.red;

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
import org.firstinspires.ftc.teamcode.drive.code.util.detection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous

public class LeftRed extends LinearOpMode {
    private DcMotorEx arm;
//    private DcMotorEx slide;
    private Servo claw;
    private Servo bclaw;
//    OpenCvCamera camera;
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
//
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

        drive.setPoseEstimate(leftRedStartPose);

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(leftRedStartPose) // increment y to go further towards blue wall
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(35, returnY(8)))
//                .addTemporalMarker(2, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), high));
//                })
                .lineToSplineHeading(new Pose2d(pickX, returnY(pickY), Math.toRadians(returnHead(pickHead))))
//                .addTemporalMarker(2, () -> {
//                    bclaw.setPosition(0.92);
//                })
                .waitSeconds(2.5) //bucket drop
//                .addTemporalMarker(2, () -> {
//                    bclaw.setPosition(0);
//                })
                .lineToSplineHeading(new Pose2d(12, returnY(12), Math.toRadians(-90)))
//                .addTemporalMarker(8.5, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), low));
//                })
                .build();

        TrajectorySequence t2 = drive.trajectorySequenceBuilder(leftRedStartPose) // increment y to go further towards blue wall
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(35, returnY(8)))
//                .addTemporalMarker(2, () -> {
//                slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), high));
//                })
                .lineToSplineHeading(new Pose2d(pickX, returnY(pickY), Math.toRadians(returnHead(pickHead))))
//                .addTemporalMarker(2, () -> {
//                bclaw.setPosition(0.92);
//                })
                .waitSeconds(2.5) //bucket drop
//                .addTemporalMarker(2, () -> {
//                bclaw.setPosition(0);
//                })
                .lineToSplineHeading(new Pose2d(35, returnY(12.5), Math.toRadians(-90)))
//                .addTemporalMarker(8.5, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), low));
//                })
                .build();

        TrajectorySequence t3 = drive.trajectorySequenceBuilder(leftRedStartPose) // increment y to go further towards blue wall
                .waitSeconds(1) // detect
                .lineTo(new Vector2d(35, returnY(8)))
//                .addTemporalMarker(2, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), high));
//                })
                .lineToSplineHeading(new Pose2d(pickX, returnY(pickY), Math.toRadians(returnHead(pickHead))))
//                .addTemporalMarker(2, () -> {
//                    bclaw.setPosition(0.92);
//                })
                .waitSeconds(2.5) //bucket drop
//                .addTemporalMarker(2, () -> {
//                    bclaw.setPosition(0);
//                })
                .lineToSplineHeading(new Pose2d(35, returnY(12), Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(35, returnY(35), Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(60, returnY(35), Math.toRadians(-90)))
//                .addTemporalMarker(8.5, () -> {
//                    slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), low));
//                })
                .build();

        waitForStart();
        drive.followTrajectorySequence(t1);
    }
    public static double returnX(double x) {
        return x * (-1);
    }
    public static double returnHead(double h) { h = Math.abs(h); return h -= 180; }
    public static double returnHead(double h, int i) { h = Math.abs(h); return h -= 360; }
    public static double returnHead(double h, String s) { h = Math.abs(h); return h -= 180; }
    public static double returnY(double y) { return y * (-1); }
}