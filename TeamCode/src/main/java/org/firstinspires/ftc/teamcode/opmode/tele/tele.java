package org.firstinspires.ftc.teamcode.opmode.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TwoWheelTrackingLocalizer;


@TeleOp(name = "W TELE")
public class tele extends LinearOpMode {
    public DcMotorEx intake;
    double btime;

    double clawClose = 0.3;
    boolean save = false;

    int targetPos = 0;
    int slideTarget = 0;
    private double loopTime;

    @Override
    public void runOpMode() {
//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        PhotonCore.enable();
        SampleMecanumDrive drive = new SampleMecanumDrive(true, hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.setPoseEstimate(PoseStorage.currentPose);
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
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


            if (gamepad1.right_bumper) { //push
                intake.setDirection(DcMotor.Direction.FORWARD);
                intake.setPower(0.8);
            }
            if (gamepad1.left_bumper) { //grab
                intake.setDirection(DcMotor.Direction.REVERSE);
                intake.setPower(0.8);
            }
        }
    }
}