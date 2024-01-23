package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PushOnePixelSlowAuto extends SequentialCommandGroup {
    public PushOnePixelSlowAuto(IntakeSubsystem intake) {
        super(
                new RunCommand(intake::pushSlow, intake).raceWith(new WaitCommand(300))
        );
    }
}
