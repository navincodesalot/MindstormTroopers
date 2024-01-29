package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

public class LiftSlideSmall extends ConditionalCommand {
    public LiftSlideSmall (DropSubsystem drop, IntakeSubsystem intake) {
        super (
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new DelayedCommand(new InstantCommand(drop::setupTrayForSlide), 75),
                                new RunCommand(intake::grab, intake).raceWith(new WaitCommand(369)).andThen(new InstantCommand(intake::stop, intake)),
                                new InstantCommand(drop::slidePoint)
                        ),
                        new WaitUntilCommand(() -> (drop.getPosition() <= 225) && (drop.getPosition() >= 180)),
                        new InstantCommand(drop::slideSmall) // 650
                ),
                new InstantCommand(drop::slideSmall),
                ()-> drop.getPosition() <= 350
        );
    }
}