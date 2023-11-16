package org.firstinspires.ftc.teamcode.opmode.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;


@TeleOp(name = "W TELE")
public class tele extends LinearOpMode {

    public CRServo intake;
    public DcMotorEx perp;
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.setPoseEstimate(PoseStorage.currentPose);
        intake = hardwareMap.get(CRServo.class, "intake");
        perp = hardwareMap.get(DcMotorEx.class, "leftRear");



        waitForStart();


        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("perp ticks", perp.getCurrentPosition());
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


            if (gamepad1.right_bumper) {
                intake.setDirection(CRServo.Direction.FORWARD);
                intake.setPower(1);
            }
            if (gamepad1.left_bumper) {
                intake.setDirection(CRServo.Direction.REVERSE);
                intake.setPower(1);
            }
        }
    }
}