package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.tuning.PIDF_Slide.d;
import static org.firstinspires.ftc.teamcode.tuning.PIDF_Slide.f;
import static org.firstinspires.ftc.teamcode.tuning.PIDF_Slide.i;
import static org.firstinspires.ftc.teamcode.tuning.PIDF_Slide.p;

import com.arcrobotics.ftclib.controller.PIDController;

public class PIDFController {


    public static double returnPower(int pos, int target) {
        PIDController controller;
        controller = new PIDController(p, i , d);
        controller.setPID(p, i, d);

        double pid = controller.calculate(pos, target);

        if (Math.abs(target - pos) < 50) { // if we say go to 1000 ticks, its at 995-1005, it will brake (to save voltage)
            return 0; // set to brake
        }

        return pid + f;
    }
}
