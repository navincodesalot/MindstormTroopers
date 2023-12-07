package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.ServoLocation;

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

    // Slide
    public Command slideLift() {
        return new InstantCommand(()-> {
            this.target = 1165;
        });
    }

    public Command slideMiddle() {
        return new InstantCommand(()-> {
            this.target = 700;
        });
    }

    public Command slideIdle() {
        return new InstantCommand(()-> {
            this.target = 10;
        });
    }

    public Command slideGoTo(int target) {
        return new InstantCommand(()-> {
            this.target = target;
        });
    }

//    public Command read() { todo
//
//    }
}
