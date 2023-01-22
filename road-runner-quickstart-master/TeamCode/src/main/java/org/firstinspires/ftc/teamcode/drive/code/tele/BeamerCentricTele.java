package org.firstinspires.ftc.teamcode.drive.code.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.code.util.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.code.util.pidf.armPIDF;
import org.firstinspires.ftc.teamcode.drive.code.util.pidf.slidePIDF;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */



@TeleOp
@Disabled
public class BeamerCentricTele extends LinearOpMode {
    private DcMotorEx arm;
    private DcMotorEx slide;
    private Servo claw;
    private Servo bclaw;

    double clawClose = 0.3;
    double targetPosS = 0;
    double targetPosA = 0;
    boolean save = false;

    public enum State {
        STOP,
        INIT,
        ARM_DOWN,
        ARM_UP,
        GRAB_CONE,
        INTO_BUCKET,
        DROP_CONE,
        SLIDE_UP,
        ARM_BACK_DOWN
    }

    State currState = State.INIT;
    double waitTime1 = 1.5;
    ElapsedTime waitTimer1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        bclaw = hardwareMap.get(Servo.class, "bclaw");

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

//        static double waitTime1 = 1.5;
//        ElapsedTime waitTimer1 = new ElapsedTime();

        waitForStart();

        arm.setTargetPosition(0);
        slide.setTargetPosition(0);
        claw.setPosition(clawClose); //close claw on init

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            gp1Controller();
            gp2Controller();

            telemetry.addData("Status", save);
            telemetry.addData("Arm Current", arm.getCurrentPosition());
            telemetry.addData("Slide Current", slide.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }

    private void gp1Controller() {
//        if (gamepad1.dpad_down) {
//            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), -97)); //grab cone
//        }
//        if (gamepad1.dpad_left) {
//            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), -20)); //lift cone slightly
//        }
//        if (gamepad1.dpad_right) {
//            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), 20)); //1st post todo maybe works
//        }
//        if (gamepad1.dpad_up) {
//            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), 20)); //put into bucket
//        }

        //Adjustments
        if (gamepad1.right_bumper) {
            targetPosS += 5;
        }
        else if (gamepad1.left_bumper) {
            targetPosS -= 5;
        }
    }

    private void gp2Controller() {
        if (gamepad2.x) { //close claw
            claw.setPosition(clawClose);
        }
        if (gamepad2.b) { //open claw
            claw.setPosition(1);
        }
        if (gamepad2.dpad_left) { //drop
            bclaw.setPosition(0.92);
        }
        if (gamepad2.dpad_right) { //pickup
            bclaw.setPosition(0);
        }

        if (gamepad2.dpad_up) {
            targetPosS = 2450;
        }
        if (gamepad2.dpad_down) {
            targetPosS = 0;
        }
        if (gamepad1.left_bumper) {
            targetPosS = 0;
            claw.setPosition(1);
        }

        if (gamepad2.a) {
            targetPosA = 20;
        }

        if (gamepad2.y) {
            targetPosA = -100;
        }

        arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), targetPosA));
        slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), targetPosS));


        //Adjustments
//        if (gamepad2.right_bumper) {
//            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), (slide.getCurrentPosition() + 100)));
//        }
//        if (gamepad2.left_bumper) {
//            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), (slide.getCurrentPosition() - 100)));
//        }
    }

    private void coneLoop() {
//        int targetPos = 0;
//        if (gamepad1.y) {
//            save = true;
//            if (gamepad1.y && save) {
//                telemetry.addData("Saving", targetPos);
//                targetPos = slide.getCurrentPosition();
//                save = false;
//            }
//        }



//



            // Still have to figure out movement automatically using rr (move forward with inches)
            // Use ASYNC FSM
//            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), 0));
//            claw.setPosition(1);
//            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), -100)); //grab cone
//            claw.setPosition(clawClose);
//            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), 15)); //put into bucket
//            claw.setPosition(1);
//            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), targetPos));
//            bclaw.setPosition(0.92);
//            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), 0));
        }
    }

