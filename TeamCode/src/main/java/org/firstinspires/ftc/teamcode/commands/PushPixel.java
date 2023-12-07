package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ServoLocation;

public class PushPixel extends SequentialCommandGroup {
    public PushPixel (IntakeSubsystem intake, DropSubsystem drop) {
        super(
                new ConditionalCommand(
                        new InstantCommand(intake::push), // todo: make push run for however long its held down
                        new InstantCommand(drop::pickupPixel).alongWith(
                                new WaitCommand(500),
                                new InstantCommand(intake::push)
                        ),
                        () -> (ServoLocation.getServoLocation() == ServoLocation.ServoLocationState.PICKUP)
                )
        );
    }
}
