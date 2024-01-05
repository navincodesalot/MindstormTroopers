package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

public class LiftSlideMed extends ConditionalCommand {
    public LiftSlideMed (DropSubsystem drop) {
        super (
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new DelayedCommand(new InstantCommand(drop::setupTrayForSlide, drop), 75),
                                new InstantCommand(drop::slidePoint, drop)
                        ),
                        new WaitUntilCommand(() -> (drop.getPosition() <= 210) && (drop.getPosition() >= 195)),
                        new InstantCommand(drop::slideMed, drop) // 900
                ),
                new InstantCommand(drop::slideMed, drop),
                ()-> drop.getPosition() <= 400

        );
    }
}