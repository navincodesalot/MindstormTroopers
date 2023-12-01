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
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Tune Slide PIDF")
public class PIDF_Slide extends OpMode {
    private PIDController controller;

    public static double p = 0.0044, i = 0.0, d = 0.0, f = 0.02;

    public static int target = 0;

    private DcMotorEx rightSlide;
    private DcMotorEx leftSlide;
    private Servo leftServo;
    private Servo rightServo;

    @Override
    public void init() {
        controller = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

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

        // lift servos TODO

    }

    @Override
    public void loop() {
        int leftSlidePos = leftSlide.getCurrentPosition();
        double power = returnPower(leftSlidePos, target);

        leftSlide.setPower(power);
        rightSlide.setPower(power);

        telemetry.addData("leftSlide Pos", leftSlide.getCurrentPosition());
        telemetry.addData("rightSlide Pos", rightSlide.getCurrentPosition());
        telemetry.addData("error", Math.abs(target - leftSlidePos));
        telemetry.addData("target", target);
        telemetry.update();
    }

    public double returnPower(int pos, int target) {
        controller.setPID(p, i, d);
        double pid = controller.calculate(pos, target);
        if (Math.abs(target - pos) <= 50 && Math.abs(target - pos) >= 30) { // if we say go to 1000 ticks, its at 995-1005, it will brake (to save voltage)
            return 0; // set to brake
        }
        return pid + f;
    }
}
