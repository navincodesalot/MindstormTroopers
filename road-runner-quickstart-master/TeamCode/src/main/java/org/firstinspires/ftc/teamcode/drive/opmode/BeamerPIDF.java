package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "PIDF Tuner")
public class BeamerPIDF extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target  = 0; //todo

    private final double ticks_in_degree = 700 / 180.0; // todo

    private DcMotorEx liftMotor;

    @Override
    public void init() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDController(p, i, d);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry()); // todo

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor"); // todo

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int liftPos = liftMotor.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        liftMotor.setPower(power);

        telemetry.addData("Pos", liftPos);
        telemetry.addData("Target", target);
        telemetry.update();
    }
}
