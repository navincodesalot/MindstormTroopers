package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.ServoLocation;

public class DropSubsystem extends SubsystemBase {
    private final VoltageSensor batteryVoltageSensor;
    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private final Servo tray;
    private final Servo leftServo;
    private final Servo rightServo;
    private final double liftedTrayPos = 0.301, leftParallel = 0.333, rightParallel = 0;
    private ElapsedTime voltageTimer;
    private double voltage;
    private double slidePower = 0;
    private int target = 0;

//    private final RunMotionProfile profile = new RunMotionProfile(
//            60000, 70000, 80000,
//            0.1, 1, 0, 0.2, 1
//    ); // todo

    public DropSubsystem(DcMotorEx leftSlide, DcMotorEx rightSlide, Servo leftServo, Servo rightServo, Servo tray, VoltageSensor b) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        this.tray = tray;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.batteryVoltageSensor = b;
        target = 0; // todo: for mp, set target/goal to 0 at start
        voltageTimer = new ElapsedTime();
        voltage = batteryVoltageSensor.getVoltage();
    }

    // PIDF Loop
    @Override
    public void periodic() { // Runs in a loop while op mode is active (in the run method of scheduler class)
        if (voltageTimer.seconds() > 5) {
            voltage = batteryVoltageSensor.getVoltage();
            voltageTimer.reset();
        }

        slidePower = (PIDFController.returnPower(leftSlide.getCurrentPosition(), target)) / voltage * 12.5;

        leftSlide.setPower(slidePower);
        rightSlide.setPower(slidePower);
        super.periodic();
    }

    // Servos
    public void goParallel() {
        leftServo.setPosition(leftParallel);
        rightServo.setPosition(rightParallel);
    }

    public void liftTray() {
        leftServo.setPosition(leftParallel);
        rightServo.setPosition(rightParallel);
        tray.setPosition(liftedTrayPos);
        ServoLocation.setServoLocation(ServoLocation.ServoLocationState.LIFTED);
    }

    public void semiLiftTrayForDrop() {
        leftServo.setPosition(leftParallel);
        rightServo.setPosition(rightParallel);
        tray.setPosition(0.23);
    }

    public void dropPixel() {
        leftServo.setPosition(0.7);
        rightServo.setPosition(0.321);
        tray.setPosition(liftedTrayPos);

        ServoLocation.setServoLocation(ServoLocation.ServoLocationState.DROP);
    }

    public void pickupPixel() {
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.2);
        tray.setPosition(0.205);
        ServoLocation.setServoLocation(ServoLocation.ServoLocationState.PICKUP);
    }

    public void setupTrayForSlide() {
        leftServo.setPosition(0);
        rightServo.setPosition(0);
        tray.setPosition(0.101);
    }

    // Slide
    public void slideHigh() {
        this.target = 1000;
    }

    public void slideMed() {
        this.target = 830;
    }

    public void slideLow() {
        this.target = 680;
    }

    public void slidePoint() {
        this.target = 200;
    }

    public void slideSmall() {
        this.target = 650;
    }

    public void slideIdle() {
        this.target = 0;
    }

    public void slideGoTo(int target) {
        this.target = target;
    }

    public int getPosition() {
        return leftSlide.getCurrentPosition();
    }

//    public double getError() {
//        return target - getPosition();
//    }
//
//    public boolean isTimeDone() {
//        return profile.getProfileDuration() < profile.getCurrentTime();
//    }
//
//    public boolean isPositionDone() {
//        return Math.abs(getError()) < 10;
//    }
//
//    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk) {
//        profile.setMotionConstraints(maxVel, maxAccel, maxJerk);
//    }
//
//    public void setPIDFcoeffs(double Kp, double Ki, double Kd, double Kf, double limit) {
//        profile.setPIDFcoeffs(Kp, Ki, Kd, Kf, limit);
//    }
//
//    public double getMotionTarget() {
//        return -profile.getMotionTarget();
//    }
//
//    public double getMotionTime() {
//        return profile.getCurrentTime();
//    }
}

