package org.firstinspires.ftc.teamcode.drive.code.util;

import static org.firstinspires.ftc.teamcode.drive.code.util.returns.returnY;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class startPoses {
    public static Pose2d leftBlueStartPose = new Pose2d(35, 61, Math.toRadians(-90));
    public static Pose2d rightBlueStartPose = new Pose2d(returns.returnX(35), 61, Math.toRadians(-90));
    public static Pose2d leftRedStartPose = new Pose2d(35, returnY(61), Math.toRadians(-270));
    public static Pose2d rightRedStartPose = new Pose2d(returns.returnX(35), returnY(61), Math.toRadians(-270));

    public static double pickX = 40, pickY = 8, pickHead = -156; //subtract value to go more clockwise
    public static double dropX = 50, dropY = 12, dropHead = 0;
}
