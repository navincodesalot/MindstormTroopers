package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakePixel extends SequentialCommandGroup {
    public IntakePixel(IntakeSubsystem intake, DropSubsystem drop) {
        addCommands(
            new InstantCommand(() -> drop.pickupPixel()),
            new WaitCommand(850),
            new InstantCommand(() -> intake.grab())
        );

        addRequirements(intake, drop);
    }
}
