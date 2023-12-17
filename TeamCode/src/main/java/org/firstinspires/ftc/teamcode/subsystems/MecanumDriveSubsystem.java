package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.rrUtil.AxesSigns;

import java.util.function.DoubleSupplier;

public class MecanumDriveSubsystem extends SubsystemBase {
    private final IMU imu;
    private final MecanumDrive drive;
    public static int joystickTransformFactor = 30;
    public int slowFactor = 3;
    private double powerLimit = 1.0;

    public MecanumDriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, IMU imu) {
        this.imu = imu;
        fR.setInverted(true);
        bL.setInverted(true);
        drive = new MecanumDrive(false, fL, fR, bL, bR);
    }

    public void fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier gyroAngle) {
         drive.driveFieldCentric(joystickTransform(strafeSpeed.getAsDouble()), joystickTransform(forwardSpeed.getAsDouble()), joystickTransform(turnSpeed.getAsDouble()), gyroAngle.getAsDouble());
    }

    public void robotCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        drive.driveRobotCentric(joystickTransform(strafeSpeed.getAsDouble()), joystickTransform(forwardSpeed.getAsDouble()), joystickTransform(turnSpeed.getAsDouble()), false);
    }

    public void slowMode(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
                drive.driveRobotCentric(joystickTransform(strafeSpeed.getAsDouble()) / slowFactor, joystickTransform(forwardSpeed.getAsDouble()) / slowFactor, joystickTransform(turnSpeed.getAsDouble()) / slowFactor);
    }

    // desmos: https://www.desmos.com/calculator/j2e6yaorld
    public double joystickTransform(double input) {
        return (1.0 / (joystickTransformFactor - 1)) * Math.signum(input) * (Math.pow(joystickTransformFactor, Math.abs(input)) - 1);
    }

    public double getPowerLimit() {
        return powerLimit;
    }

    public void setPowerLimit(double limit) {
        if (MathUtils.clamp(Math.abs(limit), 0, 1) == powerLimit)
            return;

        powerLimit = MathUtils.clamp(Math.abs(limit), 0, 1);
        drive.setMaxSpeed(powerLimit);
    }
}