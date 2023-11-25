package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Tune Slide PIDF")
public class PIDF_Slide extends OpMode {
    private PIDController controller;

    public static double p = 0.003, i = 0.25, d = 0.001;
    public static double f = 0.0105;

    public static int target = 0;

    private DcMotorEx rightSlide;
    private DcMotorEx leftSlide;

    @Override
    public void init() {
        controller = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        int leftSlidePos = leftSlide.getCurrentPosition();
        double power = returnPower(leftSlidePos, target);

        leftSlide.setPower(power);
        rightSlide.setPower(power);

        telemetry.addData("leftSlide Pos", leftSlidePos);
        telemetry.addData("rightSlide Pos", rightSlide.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.update();
    }

    public double returnPower(int pos, int target) {
        controller.setPID(p, i, d);
        double pid = controller.calculate(pos, target);

        return pid + f;
    }
}
