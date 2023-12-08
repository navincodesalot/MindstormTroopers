package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveSlowCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final SampleMecanumDrive rrDrive;
    private final GamepadEx g1;
    private final Pose2d heading;

    public DriveSlowCommand(MecanumDriveSubsystem drive, SampleMecanumDrive rrDrive, GamepadEx g1, Pose2d heading) {
        this.drive = drive;
        this.rrDrive = rrDrive;
        this.g1 = g1;
        this.heading = heading;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.slowMode(rrDrive, g1, heading);
    }
}