package org.firstinspires.ftc.teamcode.drive.code;

import android.transition.Slide;

import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.arcrobotics.ftclib.controller.PIDController;


@TeleOp
public class Test extends LinearOpMode {
    private BNO055IMU IMU;
    private DcMotorEx front_right;
    private DcMotorEx back_right;
    private DcMotorEx front_left;
    private DcMotorEx back_left;
    private DcMotorEx arm;
    private DcMotorEx slide;
    private Servo claw;
    private Servo bclaw;

    int Fast;

    /**.0
     * Describe this function...
     */

    private void Telemetry() {
        telemetry.addData("Front Right", front_right.getCurrentPosition());
        telemetry.addData("Back Right", back_right.getCurrentPosition());
        telemetry.addData("Front Left", front_left.getCurrentPosition());
        telemetry.addData("Back Left", back_left.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        PhotonCore.enable();
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        back_right = hardwareMap.get(DcMotorEx.class, "back_right");
        front_left = hardwareMap.get(DcMotorEx.class, "front_left");
        back_left = hardwareMap.get(DcMotorEx.class, "back_left");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        bclaw = hardwareMap.get(Servo.class, "bclaw");
        front_right.setDirection(DcMotorEx.Direction.FORWARD);
        back_right.setDirection(DcMotorEx.Direction.FORWARD);
        front_left.setDirection(DcMotorEx.Direction.REVERSE);
        back_left.setDirection(DcMotorEx.Direction.REVERSE);
//        claw.setPosition(0.3); //close claw on init
        //Higher number is further down and vice versa
        waitForStart();
        while (opModeIsActive()) {
            Telemetry();
        }
    }
}