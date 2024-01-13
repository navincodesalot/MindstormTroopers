package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

public class LiftSlideLow extends ConditionalCommand {
    public LiftSlideLow (DropSubsystem drop) {
        super (
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new DelayedCommand(new InstantCommand(drop::setupTrayForSlide), 75),
                                new InstantCommand(drop::slidePoint)
                        ),
                        new WaitUntilCommand(() -> (drop.getPosition() <= 215) && (drop.getPosition() >= 185)),
                        new InstantCommand(drop::slideLow) // 750
                ),
                new InstantCommand(drop::slideLow),
                ()-> drop.getPosition() <= 350
        );
    }
}