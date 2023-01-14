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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Disabled
@TeleOp(name = "PIDF Tuner")
public class BeamerPIDF extends OpMode {
    private PIDController controller;

    public static double p = 0.0095, i = 0, d = 0.0009, f = -0.15;

    public static int target  = -100; // change to see effect
    final double ticks_in_degrees = 537.7 / 180.0; // or divided by 360
    private DcMotorEx arm;

    @Override
    public void init() {
        PhotonCore.enable();
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    //this function is used to tune PIDF
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int state = arm.getCurrentPosition();

        double pid = controller.calculate(state, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;
        arm.setPower(power);

        telemetry.addData("Pos", state);
        telemetry.addData("Target", target);
        telemetry.update();
    }

    //this function will actually be used to return power to both motors
    // Ex:
    //      arm.setPower(returnPower(arm.getCurrentPosition(), target));

    public double returnPower(double state, double target) {
        controller.setPID(p, i, d);
        double pid = controller.calculate(state, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;

        arm.setPower(power);

        arm.setPower(power);

        telemetry.addData("Pos", state);
        telemetry.addData("Target", target);
        telemetry.update();
        return power;
    }
}
