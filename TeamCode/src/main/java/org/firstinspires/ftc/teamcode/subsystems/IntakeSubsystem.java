package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSubsystem extends SubsystemBase {
    private final AnalogInput intake;

    public IntakeSubsystem(AnalogInput axon) {
        this.intake = axon;
    }

//    public void grab() {
//        intake.
//    }
//
//    public void push() {
//        intake.setDirection(DcMotorEx.Direction.FORWARD);
//        intake.setPower(1);
//    }
//
//    public void pushSlow(double p) {
//        intake.setDirection(DcMotorEx.Direction.FORWARD);
//        intake.setPower(p);
//    }
//
//    public void stop() {
//        intake.setPower(0);
//    }
}
