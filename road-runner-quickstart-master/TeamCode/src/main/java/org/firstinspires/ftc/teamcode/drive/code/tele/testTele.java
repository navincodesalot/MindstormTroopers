package org.firstinspires.ftc.teamcode.drive.code.tele;

import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armDrop;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armPickup;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armStatic;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armTarget;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armTresh;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.slideTresh;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
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

@TeleOp(name = "W TELE")
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
        RESET_SLIDE
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
    int slideTarget = 0;
    private double loopTime;

    @Override
    public void runOpMode() {
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
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
        claw.setPosition(clawClose); //close claw on init++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
                                    -gamepad1.left_stick_y * 0.60,
                                    -gamepad1.left_stick_x * 0.60,
                                    -gamepad1.right_stick_x * 0.60
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
                        slideTarget = 2350;
                    }
                    if (gamepad2.left_bumper) {
                        slideTarget = 100;
                    }
                    if (gamepad2.x) { //close claw
                        claw.setPosition(clawClose);
                    }
                    if (gamepad2.b) { //open claw
                        claw.setPosition(1);
                    }
                    if (gamepad2.left_stick_button) {
                        armTarget = armStatic;
                    }
                    if (gamepad2.a) {
                        armTarget = armPickup;
                    }
                    if (gamepad2.y) {
                        armTarget = armDrop;
                    }
                    if (gamepad1.x) {
                        currentMode = Mode.CYCLE;
                        autoCurrentMode = AutoMode.INIT;
                    }
                    break;
                case CYCLE:
                        telemetry.addData("Saving", targetPos);
//                        targetPos = slide.getCurrentPosition();
                    targetPos = 2350;

                    switch (autoCurrentMode) {
                        case INIT:
                            if (gamepad1.b && targetPos != 0) {
                                arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));
                                autoCurrentMode = AutoMode.GRAB_CONE;
                            }
                            break;
                        case GRAB_CONE:
                                armTarget = armPickup; //grab cone
                                claw.setPosition(1); //open claw
                                autoCurrentMode = AutoMode.PICK_CONE;
                            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), slideTarget));
                            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));
                                break;
                        case PICK_CONE:
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < armTresh) {
                                claw.setPosition(clawClose); //close claw
                                autoCurrentMode = AutoMode.LIFT_ARM;
                            }
                            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), slideTarget));
                            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));
                            break;
                        case LIFT_ARM:
                                armTarget = armDrop;
                            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));
                            autoCurrentMode = AutoMode.INTO_BUCKET;
                                break;
                        case INTO_BUCKET:
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < armTresh) {
                                claw.setPosition(1); //open claw
                                autoCurrentMode = AutoMode.BACK_DOWN_ARM;
                                arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));
                            }
                            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), slideTarget));
                            break;
                        case BACK_DOWN_ARM:
                                armTarget = armPickup; //grab cone
                            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));
                            autoCurrentMode = AutoMode.SLIDE_UP;
                                break;

                        case SLIDE_UP:
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < armTresh) {
                                slideTarget = targetPos;
                                slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), slideTarget));
                                claw.setPosition(clawClose); //close claw
                                autoCurrentMode = AutoMode.DROP_CONE;
                            }
                            break;
                        case DROP_CONE:
                            if ((Math.abs(slide.getCurrentPosition() - slideTarget) < slideTresh)) {
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
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < armTresh) {
                                bclaw.setPosition(0); //reset
                                autoCurrentMode = AutoMode.RESET_SLIDE;
                            }
                            break;
                        case RESET_SLIDE:
                            slideTarget = 0; // todo: look into combining the reset slide and reset bucket for more time
                            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), slideTarget));
                            autoCurrentMode = AutoMode.LIFT_ARM;
                            break;
                    }
                if (gamepad1.right_bumper) {
                    currentMode = Mode.DRIVER_CONTROL;
                    break;
                }
            }
        }
    }
}
