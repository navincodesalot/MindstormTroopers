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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.rrUtil.AxesSigns;

import java.util.function.DoubleSupplier;

public class MecanumDriveSubsystem extends SubsystemBase {
    private final IMU imu;
    private final MecanumDrive drive;

    public MecanumDriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, IMU imu) {
        this.imu = imu;
        fR.setInverted(true);
        bL.setInverted(true);
        drive = new MecanumDrive(false, fL, fR, bL, bR);
    }

    public void fieldCentric(SampleMecanumDrive rrDrive, GamepadEx g1, Pose2d heading) {
            Vector2d input = new Vector2d(
                -g1.getLeftY(),
                -g1.getLeftX()
            ).rotated(-heading.getHeading());

            rrDrive.setWeightedDrivePower(
                    new Pose2d(
                        input.getX(),
                        input.getY(),
                        -g1.getRightX()
                    )
            );
    }

    public void slowMode(SampleMecanumDrive rrDrive, GamepadEx g1, Pose2d heading) {
        Vector2d input = new Vector2d(
                -g1.getLeftY(),
                -g1.getLeftX()
        ).rotated(-heading.getHeading());

        rrDrive.setWeightedSlowDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -g1.getRightX()
                ),
                3
        );
    }
}