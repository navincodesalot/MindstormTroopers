package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.util.ServoLocation;

public class LiftSlide extends SequentialCommandGroup {
    public LiftSlide (DropSubsystem drop) {
        super(
                new ConditionalCommand(
                        new InstantCommand(drop::slideLift),
                        new InstantCommand(drop::pickupPixel).alongWith(
                                new WaitCommand(400),
                                new InstantCommand(drop::slideLift)
                        ),
                        () -> (ServoLocation.getServoLocation() == ServoLocation.ServoLocationState.PICKUP)
                )
        );
    }
}
