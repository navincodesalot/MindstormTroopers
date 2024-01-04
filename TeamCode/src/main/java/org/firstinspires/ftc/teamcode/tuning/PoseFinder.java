package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Disabled
@Config
@TeleOp(name="Pose Finder")
public class PoseFinder extends OpMode {

    SampleMecanumDrive drive;
    public static double startingX = 0.0;
    public static double startingY = 0.0;
    public static double startingHeadingDegrees = 0.0;
    GamepadEx gamepadEx1;

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx1.readButtons();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Servo leftServo = hardwareMap.get(Servo.class, "leftServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightServo");
        Servo tray = hardwareMap.get(Servo.class, "tray");
        tray.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setPosition(0.333);
        rightServo.setPosition(0);
        tray.setPosition(0.301);
    }

    @Override
    public void start() {
        drive.setPoseEstimate(new Pose2d(startingX, startingY, Math.toRadians(startingHeadingDegrees)));
    }

    @Override
    public void loop() {
        gamepadEx1.readButtons();

        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
            drive.setPoseEstimate(new Pose2d(startingX, startingY, Math.toRadians(startingHeadingDegrees)));
            drive.setDrivePower(new Pose2d(0, 0, 0));
        } else {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
        }
        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}