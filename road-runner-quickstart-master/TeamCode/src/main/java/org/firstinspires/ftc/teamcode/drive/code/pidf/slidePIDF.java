package org.firstinspires.ftc.teamcode.drive.code.pidf;

import com.arcrobotics.ftclib.controller.PIDController;

public class slidePIDF {
    public static double returnPower(double state, double target) {
        double p = 0.07, i = 0.3, d = 0.00085, f = 0.04;
        PIDController controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        double pid = controller.calculate(state, target);
        double power = pid + f;
        return power;
    }
}
