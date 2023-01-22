package org.firstinspires.ftc.teamcode.drive.code.util.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.code.util.detection.AprilTagsUtil;

@Autonomous(group="Test Camera")
public class SignalDetectionAprilTest extends LinearOpMode {

    @Override
    public void runOpMode( ) throws InterruptedException {
        AprilTagsUtil detector = new AprilTagsUtil( hardwareMap, "Webcam 1", telemetry );
        detector.init();

        telemetry.addLine("camera is ready");
        telemetry.update();

        waitForStart();
    }

}