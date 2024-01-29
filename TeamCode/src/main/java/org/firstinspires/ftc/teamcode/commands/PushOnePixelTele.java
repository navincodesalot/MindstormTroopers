package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PushOnePixelTele extends SequentialCommandGroup {
    public PushOnePixelTele(IntakeSubsystem intake) {
        super(
                new RunCommand(intake::pushSlowTele, intake).raceWith(new WaitCommand(369))
        );
    }
}