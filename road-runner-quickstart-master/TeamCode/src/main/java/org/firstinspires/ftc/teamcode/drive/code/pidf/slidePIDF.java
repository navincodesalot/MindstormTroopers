package org.firstinspires.ftc.teamcode.drive.code.pidf;

import com.arcrobotics.ftclib.controller.PIDController;

public class slidePIDF {
    public static double returnPower(double state, double target) {
        double p = 0.06, i = 0.13, d = 0.0001, f = 0.04;
        PIDController controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        double pid = controller.calculate(state, target);
        double power = pid + f;
        return power;
    }
}
