package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.function.DoubleSupplier;

public class MecanumDriveSubsystem extends SubsystemBase {
    private final RevIMU imu;
    private final MecanumDrive drive;
    public double slowFactor = 3; // todo

    public MecanumDriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, RevIMU imu) {
        this.imu = imu;
        fR.setInverted(true);
        bL.setInverted(true);
        drive = new MecanumDrive(false, fL, fR, bL, bR);
    }

    public Command fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier gyroAngle, boolean clip) {
        return new RunCommand(
                () -> drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                        turnSpeed.getAsDouble(), gyroAngle.getAsDouble(), clip),
                this
        );
    }

    public Command slowMode(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> drive.driveRobotCentric(strafeSpeed.getAsDouble() / slowFactor,
                        forwardSpeed.getAsDouble() / slowFactor,
                        turnSpeed.getAsDouble() / slowFactor),
                this
        );
    }

    private double getYaw() {
        return imu.getHeading();
    }
}
