package org.firstinspires.ftc.teamcode.drive.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Beamy extends LinearOpMode {
    @Override
    public void runOpMode() {
        PhotonCore.enable();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double blueX = 35;
        double redX = returnXCoord(blueX);

        double blueY = 61;
        double redY = returnYCoord(blueY);
        Pose2d startPose = new Pose2d(redX, blueY, Math.toRadians(-90));
//        Pose2d startPose = new Pose2d(blueX, blueY, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose) // increment y to go further towards blue wall
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-35, -5)) // 2
                .lineToSplineHeading(new Pose2d(-72, -5, Math.toRadians(0))) // 1
                .lineToSplineHeading(new Pose2d(5, -5, Math.toRadians(230))) // 3

//                .lineTo(new Vector2d(35, -5)) // 2
//                .lineToSplineHeading(new Pose2d(77, -5, Math.toRadians(180))) // 1
//                .lineToSplineHeading(new Pose2d(-5, -5, Math.toRadians(315))) // 3




                // opp coterminal ang

                .build();
        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
            PoseStorage.currentPose = drive.getPoseEstimate();
    }
    public static double returnXCoord(double x) {
        return x * (-1);
    }

    public static double returnYCoord(double y) {
        return y * (-1);
    }
}

