package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.function.DoubleSupplier;

public class MecanumDriveSubsystem extends SubsystemBase {
    private final IMU imu;
    private final MecanumDrive drive;
    public static int joystickTransformFactor = 30;
    public int slowFactor = 2;
    private double powerLimit = 1.0;

    public MecanumDriveSubsystem(Motor fL, Motor fR, Motor bL, Motor bR, IMU imu) {
        this.imu = imu;
        drive = new MecanumDrive(false, fL, fR, bL, bR);
    }

    public void fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier heading) {
         drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(), turnSpeed.getAsDouble(), heading.getAsDouble(), true);
    }

    public void fieldCentric(double st, double fwd, double rot, double heading) {
        drive.driveFieldCentric(st, fwd, rot, heading, true);
    }

    public void robotCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        drive.driveRobotCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(), turnSpeed.getAsDouble(), true);
    }

    public void slowMode(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        drive.driveRobotCentric(strafeSpeed.getAsDouble() / slowFactor, forwardSpeed.getAsDouble() / slowFactor, turnSpeed.getAsDouble() / slowFactor, true);
    }

    // desmos: https://www.desmos.com/calculator/j2e6yaorld
    public double joystickTransform(double input) { // todo
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