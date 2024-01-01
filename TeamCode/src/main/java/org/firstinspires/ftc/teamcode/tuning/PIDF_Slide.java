package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.RunMotionProfile;

@Config
//@Disabled
@TeleOp(name = "Tune Slide PIDF")
public class PIDF_Slide extends OpMode {
    private PIDController controller;

    public static double p = 0.031, i = 0.31, d = 0.00046, f = 0.0048;
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
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        // lift servos
        leftServo.setPosition(0.333);
        rightServo.setPosition(0);
    }

    @Override
    public void loop() {
//        double power = profile.profiledMovement(target, getPosition());
        double power = returnPower(getPosition(), target);

        leftSlide.setPower(power);
        rightSlide.setPower(power);

        telemetry.addData("leftSlide Pos", leftSlide.getCurrentPosition());
        telemetry.addData("rightSlide Pos", rightSlide.getCurrentPosition());
        telemetry.addData("error", Math.abs(target - getPosition()));
        telemetry.addData("target", target);
        telemetry.update();
    }

    public int getPosition() {
        return leftSlide.getCurrentPosition();
    }

    public double returnPower(int pos, int target) {
        controller.setPID(p, i, d);
        double pid = controller.calculate(pos, target);
        //if (Math.abs(target - pos) <= 90 && Math.abs(target - pos) >= 50) { // if we say go to 1000 ticks, its at 995-1005, it will brake (to save voltage)
            //return 0; // set to brake
        //}
        return pid + f;
    }
}
