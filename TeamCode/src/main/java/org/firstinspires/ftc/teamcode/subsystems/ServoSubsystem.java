package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ServoLocation;

public class ServoSubsystem extends SubsystemBase {
    private final Servo leftServo;
    private final Servo rightServo;

    public ServoSubsystem(Servo leftServo, Servo rightServo) {
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }
    public Command liftServo() {
        return new InstantCommand(()-> {
            leftServo.setPosition(0.455);
            rightServo.setPosition(0.085);
            ServoLocation.setServoLocation(ServoLocation.ServoLocationState.LIFTED);
        });
    }

    public Command pickupPixel() {
        return new InstantCommand(()-> {
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.125);
            ServoLocation.setServoLocation(ServoLocation.ServoLocationState.PICKUP);
        });
    }

    public Command dropLeftPixel() { // drop pixels
        return new InstantCommand(()-> {
            rightServo.setPosition(0.53);
            ServoLocation.setServoLocation(ServoLocation.ServoLocationState.DROP_LEFT);
        });
    }

    public Command dropRightPixel() { // drop pixels
        return new InstantCommand(()-> {
            leftServo.setPosition(0.9);
            ServoLocation.setServoLocation(ServoLocation.ServoLocationState.DROP_RIGHT);
        });
    }
}
