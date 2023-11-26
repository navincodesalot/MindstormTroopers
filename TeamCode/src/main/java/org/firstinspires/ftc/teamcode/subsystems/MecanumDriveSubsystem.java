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

    public MecanumDriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, RevIMU imu) {
        this.imu = imu;
        fR.setInverted(true);
        bL.setInverted(true);
        drive = new MecanumDrive(false, fL, fR, bL, bR);
    }

    public Command fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier gyroAngle) {
        return new RunCommand(
                () -> drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                        turnSpeed.getAsDouble(), gyroAngle.getAsDouble()),
                this
        );
    }

    private double getYaw() {
        return imu.getHeading();
    }
}
