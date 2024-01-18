package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.Servo;

public class DroneSubsystem extends SubsystemBase {
    private final Servo drone;

    public DroneSubsystem(Servo drone) {
        this.drone = drone;
    }

    public void init() {
        drone.setPosition(0);
    }

    public void fly() {
        drone.setPosition(1);
    }
}