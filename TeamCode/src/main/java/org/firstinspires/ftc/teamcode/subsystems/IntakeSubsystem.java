package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor intake;

    public IntakeSubsystem(DcMotor intake) {
        this.intake = intake;
    }

    public Command grab() {
        return new InstantCommand(()-> {
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setPower(0.8);
        });
    }

    public Command push() {
        return new InstantCommand(()-> {
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setPower(0.8);
        });
    }

    public Command stop() {
        return new RunCommand(()-> {
            intake.setPower(0);
        }, this);
    }
}
