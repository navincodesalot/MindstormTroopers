package org.firstinspires.ftc.teamcode.drive.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.code.pidf.armPIDF;
import org.firstinspires.ftc.teamcode.drive.code.pidf.slidePIDF;

/**
 * This opmode demonstrates how to create a teleop using just the SampleMecanumDrive class without
 * the need for an external robot class. This will allow you to do some cool things like
 * incorporating live trajectory following in your teleop. Check out TeleOpAgumentedDriving.java for
 * an example of such behavior.
 * <p>
 * This opmode is essentially just LocalizationTest.java with a few additions and comments.
 */

@TeleOp
public class BeamerTeleOpDrive extends LinearOpMode {
    private DcMotorEx arm;
    private DcMotorEx slide;
    private Servo claw;
    private Servo bclaw;
    double target = 0;
    double high = 2650;
    double low = 35;
    double clawClose = 0.3;
    boolean save = false;

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        waitForStart();

        arm.setTargetPosition(0);
        slide.setTargetPosition(0);
        claw.setPosition(clawClose); //close claw on init

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), target));
            if (gamepad2.dpad_up) {
                target = high;
            }
            if (gamepad2.dpad_down) {
                target = low;
            }

            if (gamepad2.b) { //close claw
                claw.setPosition(clawClose);
            }
            if (gamepad2.x) { //open claw
                claw.setPosition(1);
            }
            if(gamepad2.y) { //drop
                bclaw.setPosition(0.92);
            }
            if(gamepad2.a) { //pickup
                bclaw.setPosition(0);
            }
//            gp1Controller();
//            gp2Controller();
//            coneLoop();


            Pose2d poseEstimate = drive.getPoseEstimate();
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
        if (gamepad1.dpad_down) {
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), -130)); //grab cone
        }
        if (gamepad1.dpad_left) {
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), -100)); //lift cone slightly
        }
        if (gamepad1.dpad_right) {
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), 20)); //1st post todo maybe works
        }
        if (gamepad1.dpad_up) {
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), 20)); //put into bucket
        }

        //Adjustments
        if (gamepad1.right_bumper) {
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), (arm.getCurrentPosition() + 5)));
        }
        if (gamepad1.left_bumper) {
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), (arm.getCurrentPosition() - 5)));
        }
    }

    private void gp2Controller() {
        if (gamepad2.b) { //close claw
            claw.setPosition(clawClose);
        }
        if (gamepad2.x) { //open claw
            claw.setPosition(1);
        }
        if(gamepad2.y) { //drop
            bclaw.setPosition(0.92);
        }
        if(gamepad2.a) { //pickup
            bclaw.setPosition(0);
        }

        if (gamepad2.dpad_up) {
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), 2450)); //top
        }
        if (gamepad2.dpad_down) {
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), 0)); //down to pickup
        }

        //Adjustments
        if (gamepad2.right_bumper) {
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), (slide.getCurrentPosition() + 100)));
        }
        if (gamepad2.left_bumper) {
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), (slide.getCurrentPosition() - 100)));
        }
    }

    private void coneLoop() {
        int targetPos = 0;
        if (gamepad1.y) {
            save = true;
            if (gamepad1.y && save) {
                telemetry.addData("Saving", targetPos);
                targetPos = slide.getCurrentPosition();
                save = false;
            }
        }
        // todo
        if (gamepad1.a && targetPos != 0) {
            // Still have to figure out movement automatically using rr (move forward with inches)
            // Use ASYNC FSM
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), 0));
            claw.setPosition(1);
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), -130)); //grab cone
            claw.setPosition(clawClose);
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), 20)); //put into bucket
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), targetPos));
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), 0));
        }
    }
}
