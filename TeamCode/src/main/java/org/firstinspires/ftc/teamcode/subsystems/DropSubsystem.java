package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.ServoLocation;

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
//        target = 0 // todo: for mp, set target/goal to 0 at start
    }

    //PIDF Loop
//    @Override
//    public void periodic() { //Runs in a loop while op mode is active (in the run method of scheduler class)
//        leftSlide.setPower(PIDFController.returnPower(leftSlide.getCurrentPosition(), target));
//        rightSlide.setPower(PIDFController.returnPower(leftSlide.getCurrentPosition(), target));
//        super.periodic();
//    }

    public void setPower(double p) {
        leftSlide.setPower(p);
        rightSlide.setPower(p);
    }

    // Servo
    public void liftServo() {
        leftServo.setPosition(0.455);
        rightServo.setPosition(0.085);
        ServoLocation.setServoLocation(ServoLocation.ServoLocationState.LIFTED);
    }

    public void pickupPixel() {
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.125);
        ServoLocation.setServoLocation(ServoLocation.ServoLocationState.PICKUP);
    }

    public void dropLeftPixel() {
        rightServo.setPosition(0.53);
        ServoLocation.setServoLocation(ServoLocation.ServoLocationState.DROP_LEFT);
    }

    public void dropRightPixel() {
        leftServo.setPosition(0.9);
        ServoLocation.setServoLocation(ServoLocation.ServoLocationState.DROP_RIGHT);
    }

    // Slide
    public void slideLift() {
        this.target = 1165;
    }

    public void slideMiddle() {
        this.target = 700;
    }

    public void slideIdle() {
        this.target = 10;
    }

    public void slideGoTo(int target) {
        this.target = target;
    }

    public int getPosition() {
        return leftSlide.getCurrentPosition();
    }
}
