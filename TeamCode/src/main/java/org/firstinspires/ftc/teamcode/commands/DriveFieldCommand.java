package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.ServoLocation;
import java.util.function.DoubleSupplier;

public class DriveFieldCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final DropSubsystem drop;
    private final DoubleSupplier forwardSpeed, strafeSpeed, turnSpeed, gyroAngle;
    private final boolean clip;

    public DriveFieldCommand(MecanumDriveSubsystem drive, DropSubsystem drop, DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier gyroAngle, boolean clip) {
        this.drive = drive;
        this.drop = drop;
        this.strafeSpeed = strafeSpeed;
        this.forwardSpeed = forwardSpeed;
        this.turnSpeed = turnSpeed;
        this.gyroAngle = gyroAngle;
        this.clip = clip;
        addRequirements(drive, drop);
    }

    @Override
    public void execute() {
        new ConditionalCommand(
                drive.fieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle, clip), //if true run this
                new ScheduleCommand( // else false, run this
                        drop.liftServo(),
                        new WaitCommand(500),
                        drive.fieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle, clip)
                ),
                () -> (ServoLocation.getServoLocation() == ServoLocation.ServoLocationState.LIFTED)
        );

//        if (ServoLocation.getServoLocation() == ServoLocation.ServoLocationState.LIFTED) {
//            drive.fieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle, clip);
//        } else {
//            drop.liftServo();
//            new WaitCommand(500);
//            drive.fieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle, clip);
//        }
    }
}