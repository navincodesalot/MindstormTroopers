package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.PIDFController;

public class SlideSubsystem extends SubsystemBase {
    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private int target = 0;

    //import global heights here todo

    public SlideSubsystem(DcMotorEx leftSlide, DcMotorEx rightSlide) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
    }

    @Override
    public void periodic() { //Runs in a loop while op mode is active (in the run method of scheduler class)
        leftSlide.setPower(PIDFController.returnPower(leftSlide.getCurrentPosition(), target));
        rightSlide.setPower(PIDFController.returnPower(leftSlide.getCurrentPosition(), target));
        super.periodic();
    }

    public Command lift() {
        return new InstantCommand(()-> {
            this.target = 1165;
        });
    }

    public Command middle() {
        return new InstantCommand(()-> {
            this.target = 700;
        });
    }

    public Command idle() {
        return new InstantCommand(()-> {
            this.target = 0;
        });
    }

    public Command goTo(int target) {
        return new InstantCommand(()-> {
            this.target = target;
        });
    }

//    public Command read() { todo
//
//    }
}
