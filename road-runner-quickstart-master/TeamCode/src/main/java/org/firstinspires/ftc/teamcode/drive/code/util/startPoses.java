package org.firstinspires.ftc.teamcode.drive.code.util;

import static org.firstinspires.ftc.teamcode.drive.code.util.returns.returnY;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class startPoses {
    public static Pose2d leftBlueStartPose = new Pose2d(35, 61, Math.toRadians(-90));
    public static Pose2d rightBlueStartPose = new Pose2d(returns.returnX(35), 61, Math.toRadians(-90));
    public static Pose2d leftRedStartPose = new Pose2d(35, returnY(61), Math.toRadians(-270));
    public static Pose2d rightRedStartPose = new Pose2d(returns.returnX(35), returnY(61), Math.toRadians(-270));

    public static double pickX1 = 40, pickY1 = 8, pickHead1 = -151; //subtract value to go more clockwise
    public static double pickX = 38, pickY = 10, pickHead = -149;
    public static double dropX = 53.5, dropY = 9, dropHead = 0;
}
