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

    public void grab() {
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setPower(1);
    }

    public void push() {
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setPower(1);
    }

    public void pushSlow(double p) {
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setPower(p);
    }

    public void stop() {
        intake.setPower(0);
    }
}
