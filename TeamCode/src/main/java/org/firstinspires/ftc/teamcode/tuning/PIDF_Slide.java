package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
//import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.RunMotionProfile;

@Config
//@Photon
//@Disabled
@TeleOp(name = "Tune Slide PIDF")
public class PIDF_Slide extends OpMode {
    private PIDController controller;
    private IMU imu;

//    public static double p = 0.03, i = 0.2, d = 0.00015, f = 0.0048; // tele
    public static double p = 0.03, i = 0, d = 0.00015, f = 0; // auto

//    public static double maxVel = 40000, maxAccel = 50000, maxJerk = 60000;
//    private final RunMotionProfile profile = new RunMotionProfile(
//            maxVel, maxAccel, maxJerk,
//            p, i, d, f, 1
//    ); // todo
    public static int target = 0;

    private DcMotorEx rightSlide;
    private DcMotorEx leftSlide;
    private Servo leftServo;
    private Servo rightServo;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        controller.setIntegrationBounds(0.1, 0.25);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR
                )
        ));
//
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        // lift servos
        leftServo.setPosition(0.333);
        rightServo.setPosition(0);
    }

    @Override
    public void loop() {
//        double power = profile.profiledMovement(target, getPosition());
        double power = (returnPower(getPosition(), target));

        leftSlide.setPower(power);
        rightSlide.setPower(power);

        telemetry.addData("leftSlide Pos", leftSlide.getCurrentPosition());
        telemetry.addData("rightSlide Pos", rightSlide.getCurrentPosition());
        telemetry.addData("error", Math.abs(target - getPosition()));
        telemetry.addData("target", target);
        telemetry.addData("heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    public int getPosition() {
        return leftSlide.getCurrentPosition();
    }

    public double returnPower(int pos, int target) {
        controller.setPID(p, i, d);
        double pid = controller.calculate(pos, target);

        return (pid + f);
    }
}
