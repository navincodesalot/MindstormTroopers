package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.controller.PIDController;

public class PIDFController {
    private static PIDController controller;

    public static double returnPower(int pos, int target) {
        double p = 0.003, i = 0.25, d = 0.001, f = 0.0105;
        controller.setPID(p, i, d);

        double pid = controller.calculate(pos, target);

        if (Math.abs(target - pos) < 5) { // if we say go to 1000 ticks, its at 995-1005, it will brake (to save voltage)
            return 0; // set to brake
        }

        return pid + f;
    }
}
