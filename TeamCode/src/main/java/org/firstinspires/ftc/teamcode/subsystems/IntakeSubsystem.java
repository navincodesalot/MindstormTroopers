package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx intake;

    public IntakeSubsystem(DcMotorEx intake) {
        this.intake = intake;
    }

    public Command grab() {
        return new InstantCommand(()-> {
            intake.setDirection(DcMotorEx.Direction.REVERSE);
            intake.setPower(1);
        });
    }

    public Command push() {
        return new InstantCommand(()-> {
            intake.setDirection(DcMotorEx.Direction.FORWARD);
            intake.setPower(1);
        });
    }

    public Command stop() {
        return new RunCommand(()-> {
            intake.setPower(0);
        }, this); // the `this` is the intake subsystem as the requirements (only for run commands)
    }
}
