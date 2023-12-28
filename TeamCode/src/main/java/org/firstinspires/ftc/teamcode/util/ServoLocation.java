package org.firstinspires.ftc.teamcode.util;

public class ServoLocation {

    public enum ServoLocationState {
        LIFTED,
        PICKUP,
        DROP
    }

    private static ServoLocationState servoLocation;

    public static void setServoLocation(ServoLocationState sL) {
        servoLocation = sL;
    }

    public static ServoLocationState getServoLocation() {
        return servoLocation;
    }
}
