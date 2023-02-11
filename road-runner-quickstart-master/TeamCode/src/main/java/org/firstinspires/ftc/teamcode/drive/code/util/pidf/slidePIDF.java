package org.firstinspires.ftc.teamcode.drive.code.util.pidf;

import com.arcrobotics.ftclib.controller.PIDController;

public class slidePIDF {
    public static double returnPower(double state, double target) {
        double p = 0.012, i = 0.001, d = 0.000, f = 0.02; //g value https://www.ctrlaltftc.com/feedforward-control#slide-gravity-feedforward
        PIDController controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        double pid = controller.calculate(state, target);
        double power = pid + f;
        return power;
    }
}
