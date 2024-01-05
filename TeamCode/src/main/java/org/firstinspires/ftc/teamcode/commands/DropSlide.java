package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

public class DropSlide extends SequentialCommandGroup {
    public DropSlide (DropSubsystem drop) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(drop::setupTrayForSlide, drop),
                        new DelayedCommand(new InstantCommand(() -> drop.slideGoTo(250), drop), 750)
                ),
                new WaitUntilCommand(() -> (drop.getPosition() <= 255) && (drop.getPosition() >= 242)),
                new ParallelCommandGroup(
                        new DelayedCommand(new InstantCommand(drop::semiLiftTrayForDrop, drop), 100),
                        new DelayedCommand(new InstantCommand(drop::slideIdle, drop), 300)
                )
        );
    }
}