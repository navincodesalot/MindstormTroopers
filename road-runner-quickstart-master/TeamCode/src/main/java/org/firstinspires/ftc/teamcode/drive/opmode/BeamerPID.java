package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "PID Tuner")
public class BeamerPID extends LinearOpMode {
    DcMotorEx liftMotor1, liftMotor2; //todo
    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    public static int targetPosition = 5000; // todo Find target pos manually

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1"); // todo
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");

        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        int targetPosition = 5000; // todo

        while(opModeIsActive()) {
            double power1 = returnPower(liftMotor1.getCurrentPosition(), targetPosition);
            double power2 = returnPower(liftMotor2.getCurrentPosition(), targetPosition);

            packet.put("Power1", power1);
            packet.put("Power2", power2);
            packet.put("Position", liftMotor1.getCurrentPosition());
            packet.put("Error", lastError);

            liftMotor1.setPower(power1);
            liftMotor2.setPower(power2);

            dashboard.sendTelemetryPacket(packet);
        }
    }
    public double returnPower(double state, double target) {
        double error = target - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

}
