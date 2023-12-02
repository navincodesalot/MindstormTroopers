package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;

public class LiftSlide extends SequentialCommandGroup {
    public LiftSlide (DropSubsystem drop) {
        addCommands(
                new InstantCommand(drop::pickupPixel),
                new WaitCommand(200),
                new InstantCommand(drop::slideLift)
        );

        addRequirements(drop);
    }
}
