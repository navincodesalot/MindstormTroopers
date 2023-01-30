package org.firstinspires.ftc.teamcode.drive.code.tele;

import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armPickup;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armStatic;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armTarget;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armTresh;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.slideTarget;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.slideTresh;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@TeleOp
public class testTele extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        CYCLE
    }

    enum AutoMode {
        INIT,
        GRAB_CONE,
        PICK_CONE,
        LIFT_ARM,
        INTO_BUCKET,
        BACK_DOWN_ARM,
        SLIDE_UP,
        DROP_CONE,
        RESET,
        RESET_SLIDE,
        AGAIN
    }

    Mode currentMode = Mode.DRIVER_CONTROL;
    AutoMode autoCurrentMode;

    public DcMotorEx arm;
    public DcMotorEx slide;
    public Servo claw;
    public Servo bclaw;

    double btime;

    double clawClose = 0.3;
    boolean save = false;

    int targetPos = 0;
    private double loopTime;

    @Override
    public void runOpMode() {
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.enable();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        arm.setTargetPosition(0);
        slide.setTargetPosition(0);
        claw.setPosition(clawClose); //close claw on init

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), slideTarget));
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();

            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * 0.6,
                                    -gamepad1.left_stick_x * 0.6,
                                    -gamepad1.right_stick_x * 0.6
                            )
                    );
                    drive.update();

                    if (gamepad1.y) { //drop
                        bclaw.setPosition(0.92);
                    }
                    if (gamepad1.a) { //pickup
                        bclaw.setPosition(0);
                    }
                    if (gamepad2.right_bumper) {
                        slideTarget = 2650;
                    }
                    if (gamepad2.left_bumper) {
                        slideTarget = 0;
                    }
                    if (gamepad2.b) { //close claw
                        claw.setPosition(clawClose);
                    }
                    if (gamepad2.x) { //open claw
                        claw.setPosition(1);
                    }
                    if (gamepad2.left_stick_button) {
                        armTarget = -20;
                    }
                    if (gamepad2.a) {
                        armTarget = armPickup;
                    }
                    if (gamepad2.y) {
                        armTarget = 20;
                    }
                    if (gamepad1.x) {
                        save = true;
                        currentMode = Mode.CYCLE;
                    }
                    break;
                case CYCLE:
                    if (gamepad1.x && save) {
                        telemetry.addData("Saving", targetPos);
                        targetPos = slide.getCurrentPosition();
                        save = false;
                        autoCurrentMode = AutoMode.INIT;
                    }
                    switch (autoCurrentMode) {
                        case INIT:
                            if (gamepad1.b && targetPos != 0) {
                                autoCurrentMode = AutoMode.GRAB_CONE;
                            }
                            break;
                        case GRAB_CONE:
                                armTarget = armPickup; //grab cone
                                claw.setPosition(1); //open claw
                                autoCurrentMode = AutoMode.PICK_CONE;
                            break;
                        case PICK_CONE:
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < armTresh) {
                                claw.setPosition(clawClose); //close claw
                                autoCurrentMode = AutoMode.LIFT_ARM;
                            }
                            break;
                        case LIFT_ARM:
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < armTresh) {
                                armTarget = 20;
                                autoCurrentMode = AutoMode.INTO_BUCKET;
                            }
                            break;
                        case INTO_BUCKET:
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < armTresh) {
                                claw.setPosition(1); //open claw
                                autoCurrentMode = AutoMode.BACK_DOWN_ARM;
                            }
                            break;
                        case BACK_DOWN_ARM:
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < armTresh) {
                                armTarget = armStatic;
                                autoCurrentMode = AutoMode.SLIDE_UP;
                            }
                            break;
                        case SLIDE_UP:
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < armTresh) {
                                slideTarget = targetPos;
                                armTarget = armPickup; //grab cone
                                claw.setPosition(1); //open claw
                                autoCurrentMode = AutoMode.DROP_CONE;
                            }
                            break;
                        case DROP_CONE:
                            if (Math.abs(slide.getCurrentPosition() - slideTarget) < slideTresh) {
                                bclaw.setPosition(0.92);
                                ElapsedTime bclawTimer = new ElapsedTime();
                                btime = 1; // 1 second wait
                                if (bclawTimer.seconds() >= btime) {
                                    bclawTimer.reset();
                                    autoCurrentMode = AutoMode.RESET;
                                }
                            }
                            break;
                        case RESET:
                            bclaw.setPosition(0); //reset
                            autoCurrentMode = AutoMode.RESET_SLIDE;
                            break;
                        case RESET_SLIDE:
                            slideTarget = 0;
                            autoCurrentMode = AutoMode.AGAIN;
                            break;
                        case AGAIN:
                            if (Math.abs(slide.getCurrentPosition() - slideTarget) < slideTresh) {
                                if (gamepad1.b) {
                                    autoCurrentMode = AutoMode.INIT;
                                }
                                autoCurrentMode = AutoMode.LIFT_ARM;
                            }
                            break;
                    }
                if (gamepad1.right_bumper) {
                    currentMode = Mode.DRIVER_CONTROL;
                }
            }
        }
    }
}
