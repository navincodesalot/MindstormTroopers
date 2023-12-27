package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RunMotionProfile {
    private double lastTarget;
    private double maxVel, maxAccel, maxJerk;
    private MotionState motionState;
    private final PIDFController PIDF;
    private final ElapsedTime timer = new ElapsedTime();
    private MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0,0,0),
            new MotionState(1,0,0),
            1,
            1,
            1
    );

    public RunMotionProfile(double maxVel, double maxAccel, double maxJerk, double Kp, double Ki, double Kd, double Kf, double limit) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;

        PIDF = new PIDFController(Kp, Ki, Kd, Kf);
    }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;
    }

    public void setPIDFcoeffs(double Kp, double Ki, double Kd, double Kf, double limit) {
        PIDF.setPIDF(Kp, Ki, Kd, Kf);
    }

    public double profiledMovement(double target, double state) {
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(state, 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel, maxJerk);
            timer.reset();
        } else {
            lastTarget = target;
        }
        motionState = profile.get(timer.seconds());
        return PIDF.calculate(motionState.getX() - state);
    }

    public double profiledServoMovement(double target, double state) { // todo: use for gobilda servos if needed
        if (lastTarget != target) {
            lastTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(state, 0, 0), new MotionState(target, 0, 0), maxVel, maxAccel, maxJerk);
            timer.reset();
        }
        else {
            lastTarget = target;
        }
        motionState = profile.get(timer.seconds());
        return motionState.getX();
    }

    public double getMotionTarget() {
        if (motionState == null) {
            return 0;
        }
        return motionState.getX();
    }

    public double getProfileDuration() {
        return profile.duration();
    }

    public double getCurrentTime() {
        return timer.seconds();
    }
}
