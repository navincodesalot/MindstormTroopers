package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PushPixel extends SequentialCommandGroup {
    public PushPixel(IntakeSubsystem intake, DropSubsystem drop) {
        addCommands(
                new InstantCommand(drop::pickupPixel),
                new WaitCommand(500),
                new InstantCommand(intake::grab)
        );

        addRequirements(intake, drop);
    }
}
