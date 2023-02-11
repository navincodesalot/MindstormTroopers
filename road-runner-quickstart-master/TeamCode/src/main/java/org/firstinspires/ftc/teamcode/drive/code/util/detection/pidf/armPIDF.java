package org.firstinspires.ftc.teamcode.drive.code.util.detection.pidf;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

public class armPIDF {
    public static double returnPower(double state, double target) {
        double p = 0.0032, i = 0.0, d = 0.0001, f = 0.14; // when going down to 130 its 5 off
        double divide = 1.2;
        final double ticks_in_degrees = 537.7 / 360.0;

        PIDController controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        double pid = controller.calculate(state, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;
        power /= divide;
        return power;
    }
}
