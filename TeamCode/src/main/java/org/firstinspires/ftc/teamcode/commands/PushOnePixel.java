package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PushOnePixel extends SequentialCommandGroup {
    public PushOnePixel(IntakeSubsystem intake) {
        super(
                new RunCommand(intake::push, intake).raceWith(new WaitCommand(150))
        );
    }
}
