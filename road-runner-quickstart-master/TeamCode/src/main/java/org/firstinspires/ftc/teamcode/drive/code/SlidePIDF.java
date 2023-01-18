package org.firstinspires.ftc.teamcode.drive.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config

@TeleOp(name = "Slide PIDF Tuner")
public class SlidePIDF extends OpMode {
    private PIDController controller;

    public static double p = 0.07, i = 0.4, d = 0.001, f = 0.04; //g value https://www.ctrlaltftc.com/feedforward-control#slide-gravity-feedforward

    public static int target  = 2750; //todo

    private DcMotorEx slide;

    @Override
    public void init() {
        PhotonCore.enable();
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        slide = hardwareMap.get(DcMotorEx.class, "slide"); // todo

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //this function is used to tune PIDF
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int state = slide.getCurrentPosition();

        double pid = controller.calculate(state, target);
        double power = pid + f;
//        double power2 = pid2 + f;

//        liftMotor1.setPower(-power);
        slide.setPower(power);

        telemetry.addData("SlidePos", state);
        telemetry.addData("Target", target);
        telemetry.addData("Pid Power", pid);
        telemetry.update();
    }

    //this function will actually be used to return power to both motors
    // Ex:
    //      liftMotor1.setPower(returnPower(liftMotor1.getCurrentPosition(), target));
    //      liftMotor2.setPower(returnPower(liftMotor2.getCurrentPosition(), target));

//    public double returnPower(double state, double target) {
//        controller.setPID(p, i, d);
//        double pid = controller.calculate(state, target);
//
//        double power = pid + f;
//
//        liftMotor1.setPower(power);
//
//        telemetry.addData("Pos", state);
//        telemetry.addData("Target", target);
//        telemetry.update();
//        return power;
//    }
}