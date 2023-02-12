package org.firstinspires.ftc.teamcode.drive.code.tele;

import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armDrop;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armPickup;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armStatic;
import static org.firstinspires.ftc.teamcode.drive.code.util.tele.armTarget;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.code.util.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.code.util.detection.pidf.armPIDF;
import org.firstinspires.ftc.teamcode.drive.code.util.detection.pidf.slidePIDF;

@TeleOp(name = "W TELE")
public class testTele extends LinearOpMode {

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
        claw.setPosition(clawClose); //close claw on init30.

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
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * 0.40,
                                    gamepad1.left_stick_x * 0.40,
                                    -gamepad1.right_stick_x * 0.40
                            )
                    );
                    drive.update();

                    if (gamepad1.y) { //drop
                        bclaw.setPosition(0.92);
                    }
                    if (gamepad1.a) { //pickup
                        bclaw.setPosition(0);
                    }
                    if (gamepad1.right_bumper) {
                        slideTarget = 2350;
                    }
                    if (gamepad1.left_bumper) {
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
        }
    }
}
