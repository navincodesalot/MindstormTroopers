package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDFController;

public class DropSubsystem extends SubsystemBase {
    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private final Servo leftServo;
    private final Servo rightServo;

    private int target = 0;

    //import global heights here todo

    public DropSubsystem(DcMotorEx leftSlide, DcMotorEx rightSlide, Servo leftServo, Servo rightServo) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }

    @Override
    public void periodic() { //Runs in a loop while op mode is active (in the run method of scheduler class)
        leftSlide.setPower(PIDFController.returnPower(leftSlide.getCurrentPosition(), target));
        rightSlide.setPower(PIDFController.returnPower(leftSlide.getCurrentPosition(), target));
        super.periodic();
    }

    // Servo
    public Command liftServo() {
        // in init we lift
        // default command is to keep them lifted,
        // to drive, we need to call lift command before we drive (cant drive unless the lift command is called)
        // when intake button is hit, we need to call the pickupPixel command, wait time, then intake
        // AND we need to brake all the drive commands (cant drive while intaking)
        // once intake is complete (timed for now, DS later), we need to lift the servos
        // when we call slide up, servos lock with the pickupPixel command
        // when we drop, it does the dropPixels command
        // when we call slide down, state of servo gets switched and it goes to lift mode

        // for all of this, make dedicated command files with sequentials and as many parallel groups I can

        return new InstantCommand(()-> {
            leftServo.setPosition(0.472);
            rightServo.setPosition(0.1);
        });
    }

    public Command pickupPixel() {
        return new InstantCommand(()-> {
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.125);
        });
    }

    public Command dropPixels() { // drop pixels
        return new InstantCommand(()-> {
            leftServo.setPosition(0.9);
            rightServo.setPosition(0.53);
        });
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
            this.target = 0;
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
