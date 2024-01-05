package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class AprilTagSubsystem extends SubsystemBase {

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal portal;
    private final WebcamName webcam1, webcam2;

    private final Vector2d camera1Offset = new Vector2d(0, 0); // todo: find in inches
    private final Vector2d camera2Offset = new Vector2d(0, -6.375);

    public AprilTagSubsystem(HardwareMap hardwareMap, String camera1Name, String camera2Name) {
        webcam1 = hardwareMap.get(WebcamName.class, camera1Name);
        webcam2 = hardwareMap.get(WebcamName.class, camera2Name);

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTagProcessor)
                .setAutoStopLiveView(true)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return new ArrayList<>();

        return aprilTagProcessor.getDetections();
    }

    public void switchCamera(String cameraName) {
        if (cameraName.equals(webcam1.getDeviceName())) {
            portal.setActiveCamera(webcam1);
        } else {
            portal.setActiveCamera(webcam2);
        }
    }

    /**
     * @param botheading In Radians.
     * @return FC Pose of bot.
     */
    public Vector2d getFCPosition(AprilTagDetection detection, double botheading) {
        Vector2d cameraOffset;
        if (portal.getActiveCamera().equals(webcam2)) {cameraOffset = camera2Offset;} else {cameraOffset = camera1Offset;}
        // get coordinates of the robot in RC coordinates
        // ensure offsets are RC
        double x = detection.ftcPose.x - cameraOffset.getX();
        double y = detection.ftcPose.y - cameraOffset.getY();

        // invert heading to correct properly
        botheading = -botheading;

        // rotate RC coordinates to be field-centric
        double x2 = x * Math.cos(botheading) + y * Math.sin(botheading);
        double y2 = x * -Math.sin(botheading) + y * Math.cos(botheading);
        double absX;
        double absY;

        // add FC coordinates to apriltag position
        VectorF tagpose = detection.metadata.fieldPosition;
        if (detection.metadata.id <= 6) { // first 6 are backdrop tags
            absX = tagpose.get(0) + y2;
            absY = tagpose.get(1) - x2;

        } else { // then just reverse positions
            absX = tagpose.get(0) - y2;
            absY = tagpose.get(1) + x2;
        }
        // Don't send over a pose, as apriltag heading can be off (see discord)
        return new Vector2d(absX, absY);
    }

    public void shutdown() {
        if (portal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED)
            return;

        portal.close();
    }
}