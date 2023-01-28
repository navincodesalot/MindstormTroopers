package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BaseMeep {
    public static Pose2d leftBlueStartPose = new Pose2d(35, 61, Math.toRadians(-90));
    public static Pose2d rightBlueStartPose = new Pose2d(returnX(35), 61, Math.toRadians(-90));
    public static Pose2d leftRedStartPose = new Pose2d(35, returnY(61), Math.toRadians(-270));
    public static Pose2d rightRedStartPose = new Pose2d(returnX(35), returnY(61), Math.toRadians(-270));

    public static double pickX1 = 40, pickY1 = 8, pickHead1 = -156; //subtract value to go more clockwise
    public static double pickX = 38, pickY = 10, pickHead = -149;
    public static double dropX = 53, dropY = 12, dropHead = 0;
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
    public static int maxVel = 30, maxAccel = 30, trackWidth = 13;

    public static int park = 1;
}