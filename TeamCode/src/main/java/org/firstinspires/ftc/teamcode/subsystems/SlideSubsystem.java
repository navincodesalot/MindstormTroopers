package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.PIDFController;

public class SlideSubsystem extends SubsystemBase {
    private final DcMotor leftSlide;
    private final DcMotor rightSlide;
    private int target = 0;

    //import global heights here todo

    public SlideSubsystem(DcMotor leftSlide, DcMotor rightSlide) {
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
            this.target = 600;
        });
    }

    public Command drop() {
        return new InstantCommand(()-> {
            this.target = 150;
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