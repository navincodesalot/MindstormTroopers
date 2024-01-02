package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;

public class LiftSlide extends SequentialCommandGroup {
    public LiftSlide (DropSubsystem drop) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(drop::setupTrayForSlide),
                        new InstantCommand(drop::slideLiftPoint)
                ),
                new WaitUntilCommand(() -> (drop.getPosition() <= 210) && (drop.getPosition() >= 190)), // figure out (or use time if needed)
                new InstantCommand(drop::slideLift)
                // do tray stuff here (after we use a wait until and check the slides pos)
        );
    }
}
