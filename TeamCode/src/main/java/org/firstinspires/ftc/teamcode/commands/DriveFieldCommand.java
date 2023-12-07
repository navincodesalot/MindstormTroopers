package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.ServoLocation;
import java.util.function.DoubleSupplier;

public class DriveFieldCommand extends SequentialCommandGroup {
    public DriveFieldCommand(MecanumDriveSubsystem drive, DropSubsystem drop, DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier gyroAngle, boolean clip) {
        super(
                new ConditionalCommand(
                    // if true run this
                    new RunCommand(
                            () -> drive.fieldCentric(
                                    strafeSpeed,
                                    forwardSpeed,
                                    turnSpeed,
                                    gyroAngle,
                                    clip
                            )
                    ),
                    // else false, run this
                    new InstantCommand(drop::liftServo).alongWith(
                            new WaitCommand(500),
                            new RunCommand(
                            () -> drive.fieldCentric(
                                    strafeSpeed,
                                    forwardSpeed,
                                    turnSpeed,
                                    gyroAngle,
                                    clip
                            )
                    )),
                    () -> (ServoLocation.getServoLocation() == ServoLocation.ServoLocationState.LIFTED)
            )
        );
    }
}