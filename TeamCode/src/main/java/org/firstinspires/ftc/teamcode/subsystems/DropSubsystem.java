package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.RunMotionProfile;
import org.firstinspires.ftc.teamcode.util.ServoLocation;

public class DropSubsystem extends SubsystemBase {
    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private final Servo leftServo;
    private final Servo rightServo;
    private int target = 0;
    private final RunMotionProfile profile = new RunMotionProfile(
            60000,70000,80000,
            0.1,1,0,0.2, 1
    ); // todo

    public DropSubsystem(DcMotorEx leftSlide, DcMotorEx rightSlide, Servo leftServo, Servo rightServo) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        target = 0; // todo: for mp, set target/goal to 0 at start
    }

    //PIDF Loop
    @Override
    public void periodic() { //Runs in a loop while op mode is active (in the run method of scheduler class)
        leftSlide.setPower(profile.profiledMovement(target, getPosition()));
        rightSlide.setPower(profile.profiledMovement(target, getPosition()));
        super.periodic();
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

    public double getError() {
        return target - getPosition();
    }

    public int getPosition() {
        return leftSlide.getCurrentPosition();
    }

    public boolean isTimeDone() {
        return profile.getProfileDuration() < profile.getCurrentTime();
    }
    public boolean isPositionDone() {
        return Math.abs(getError()) < 10;
    }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk) {
        profile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public void setPIDFcoeffs(double Kp, double Ki, double Kd, double Kf, double limit) {
        profile.setPIDFcoeffs(Kp, Ki, Kd, Kf, limit);
    }

    public double getMotionTarget() {
        return -profile.getMotionTarget();
    }

    public double getMotionTime() {
        return profile.getCurrentTime();
    }
}

