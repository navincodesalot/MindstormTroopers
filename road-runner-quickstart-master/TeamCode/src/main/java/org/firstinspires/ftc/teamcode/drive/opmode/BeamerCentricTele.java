package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp
public class BeamerCentricTele extends LinearOpMode {
    private DcMotorEx liftMotor1;
    private DcMotorEx liftMotor2;
    private Servo claw;
    private Servo dclaw;
    private Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");

        claw = hardwareMap.get(Servo.class, "claw");
        dclaw = hardwareMap.get(Servo.class, "dclaw");
        arm = hardwareMap.get(Servo.class, "arm");

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
     // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();
            extendLift();
            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("servo", dclaw.getPosition());
            telemetry.addData("heading in degrees", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }

    private void extendLift() {
//        //Higher number is further down and vice versa FOR DCLAW

        if (gamepad2.x) {
            claw.setPosition(0);
        }
        if (gamepad2.b) {
            claw.setPosition(1);
        }

        if (gamepad2.right_bumper){
            dclaw.setPosition(0.5);
        }

        if (gamepad2.left_bumper){
            dclaw.setPosition(0);
        }

        if(gamepad2.right_stick_button){
            arm.setPosition(0.5);
        }

        if(gamepad2.left_stick_button){
            arm.setPosition(0);
        }


        if (gamepad2.dpad_up) {
            liftMotor1.setTargetPosition(-1699);
            liftMotor2.setTargetPosition(-1746);
//            arm.setPosition(0.5);
//            dclaw.setPosition(0.5);
        }
        if (gamepad2.dpad_right) {
            liftMotor2.setTargetPosition(-1275);
            liftMotor1.setTargetPosition(-1309);
//            arm.setPosition(0);
//            dclaw.setPosition(0.5);
        }
        if (gamepad2.dpad_down) {
            liftMotor1.setTargetPosition(0);
            liftMotor2.setTargetPosition(0);
//            arm.setPosition(0);
//            dclaw.setPosition(0);
        }
        if (gamepad2.y) {
            liftMotor1.setTargetPosition(liftMotor1.getTargetPosition() - 25);
            liftMotor2.setTargetPosition(liftMotor2.getTargetPosition() - 25);
        }
        if (gamepad2.a) {
            liftMotor1.setTargetPosition(liftMotor1.getTargetPosition() + 25);
            liftMotor2.setTargetPosition(liftMotor2.getTargetPosition() + 25);
        }
        if (liftMotor1.getCurrentPosition() < liftMotor1.getTargetPosition() - 50 && liftMotor2.getCurrentPosition() < liftMotor2.getTargetPosition() - 50) {
            liftMotor1.setVelocity(-2000);
            liftMotor2.setVelocity(-2000);
            liftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        } else if (liftMotor1.getCurrentPosition() > liftMotor1.getTargetPosition() + 50 && liftMotor2.getCurrentPosition() > liftMotor2.getTargetPosition() + 50) {
            liftMotor1.setVelocity(1300);
            liftMotor2.setVelocity(1300);
            liftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }
}