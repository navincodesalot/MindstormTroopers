package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intake;

    public IntakeSubsystem(CRServo axon) {
        this.intake = axon;
    }

    public void grab() {
        intake.setDirection(CRServo.Direction.FORWARD);
        intake.setPower(1);
    }

    public void push() {
        intake.setDirection(CRServo.Direction.REVERSE);
        intake.setPower(1);
    }

    public void pushSlow(double p) {
        intake.setDirection(CRServo.Direction.REVERSE);
        intake.setPower(p);
    }

    public void stop() {
        intake.setPower(0);
    }
}
