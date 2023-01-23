package org.firstinspires.ftc.teamcode.drive.code.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
        ARM_DOWN,
        GRAB_CONE,
        SLIDE_UP,
        RESET
    }

    Mode currentMode = Mode.DRIVER_CONTROL;
    AutoMode autoCurrentMode;

    public DcMotorEx arm;
    public DcMotorEx slide;
    public Servo claw;
    public Servo bclaw;

    double btime;

    double clawClose = 0.3;
    int armTarget = 0;
    int slideTarget = 0;
    int armPickup = -100;
    boolean save = false;

    int targetPos = 0;

    @Override
    public void runOpMode() {
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
                                autoCurrentMode = AutoMode.ARM_DOWN;
                            }
                            break;
                        case ARM_DOWN:
                            slideTarget = 0; //reset to 0 in case
                            autoCurrentMode = AutoMode.GRAB_CONE;
                            break;
                        case GRAB_CONE:
                            if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) { // our threshold is within 10 encoder ticks of our target
                                claw.setPosition(1); //open claw
                                armTarget = armPickup; //grab cone
                                if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) { // our threshold is within 5 encoder ticks of our target
                                    claw.setPosition(clawClose); //close claw
                                    armTarget = 20;
                                }

                                if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) { // our threshold is within 5 encoder ticks of our target
                                    claw.setPosition(1); //open claw
                                    armTarget = -35;
                                    autoCurrentMode = AutoMode.SLIDE_UP;
                                }
                               
                            }
                            break;      
                  
                        case SLIDE_UP:
                            if (Math.abs(arm.getCurrentPosition() - armTarget) < 5) { // our threshold is within 5 encoder ticks of our target
                                slideTarget = targetPos;
                            }
                            if (Math.abs(slide.getCurrentPosition() - slideTarget) < 10) { // our threshold is within 10 encoder ticks of our target
                                bclaw.setPosition(0.92);
                                ElapsedTime bclawTimer = new ElapsedTime();
                                btime = 2; // 2 second wait
                                if (bclawTimer.seconds() >= btime) {
                                    bclawTimer.reset();
                                    autoCurrentMode = AutoMode.RESET;
                                }
                            }
                            break;
        
                        case RESET:
                            bclaw.setPosition(0); //reset
//                            if (gamepad1.x && autoCurrentMode != autoCurrentMode.INIT) {
//                                autoCurrentMode = AutoMode.INIT;
//                            }
                            if (gamepad1.b) {
                                autoCurrentMode = AutoMode.INIT;
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
