package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.CRServo;

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

    public void pushSlow() {
        intake.setDirection(CRServo.Direction.REVERSE);
        intake.setPower(0.32);
    }

    public void stop() {
        intake.setPower(0);
    }
}