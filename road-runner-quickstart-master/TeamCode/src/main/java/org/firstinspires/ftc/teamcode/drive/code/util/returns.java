package org.firstinspires.ftc.teamcode.drive.code.util;

public class returns {
    public static double returnX(double x) {
        return x * (-1);
    }
    public static double returnHead(double h) {
        h = Math.abs(h); return h += 180;
    }
    public static double returnHead(double h, int i) {
        h = Math.abs(h); return h -= 360;
    }
    public static double returnHead(double h, String s) {
        h = Math.abs(h); return h -= 180;
    }
    public static double returnY(double y) {
        return y * (-1);
    }
}
