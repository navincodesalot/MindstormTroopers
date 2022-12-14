package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "PIDF Tuner")
public class BeamerPIDF extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0; //g value https://www.ctrlaltftc.com/feedforward-control#slide-gravity-feedforward

    public static int target  = 0; //todo

    private DcMotorEx liftMotor1, liftMotor2;

    @Override
    public void init() {
        PhotonCore.enable();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDController(p, i, d);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1"); // todo
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");

        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    //this function is used to tune PIDF
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int state1 = liftMotor1.getCurrentPosition();
        int state2 = liftMotor1.getCurrentPosition();

        double pid1 = controller.calculate(state1, target);
        double pid2 = controller.calculate(state2, target);

        double power1 = pid1 + f;
        double power2 = pid2 + f;

        liftMotor1.setPower(power1);
        liftMotor2.setPower(power2);

        telemetry.addData("Pos1", state1);
        telemetry.addData("Pos2", state2);
        telemetry.addData("Target", target);
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
