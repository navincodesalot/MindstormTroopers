package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;

public class HangSlide extends ConditionalCommand {
    public HangSlide (DropSubsystem drop) {
        super (
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new DelayedCommand(new InstantCommand(drop::setupTrayForSlide), 75),
                                new InstantCommand(drop::slidePoint)
                        ),
                        new WaitUntilCommand(() -> (drop.getPosition() <= 225) && (drop.getPosition() >= 180)),
                        new InstantCommand(drop::goToHang) // 769
                ),
                new InstantCommand(drop::goToHang),
                ()-> drop.getPosition() <= 350
        );
    }
}