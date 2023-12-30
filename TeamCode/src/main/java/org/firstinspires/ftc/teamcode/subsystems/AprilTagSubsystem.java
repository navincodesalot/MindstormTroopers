package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    public VisionPortal portal;
    private List<Integer> targetsList;

    public AprilTagSubsystem(HardwareMap hardwareMap, String cameraName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .addProcessor(aprilTagProcessor)
                .setAutoStopLiveView(true)
                .build();
    }

    public AprilTagSubsystem(HardwareMap hardwareMap, String cameraName, Integer... targets) {
        this(hardwareMap, cameraName);
        this.setTargetTags(targets);
    }

    public void setTargetTags(Integer... targets) {
        targetsList = Arrays.asList(targets);
    }

    /**
     * Filters currently detected AprilTags based on the set targets.
     *
     * @return A list of AprilTag detections containing targets exclusively
     */
    public List<AprilTagDetection> getFilteredDetections() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING || targetsList.isEmpty())
            return new ArrayList<>();

        List<AprilTagDetection> aprilTagDetections = aprilTagProcessor.getDetections();
        return aprilTagDetections.stream()
                .filter(detection -> targetsList.contains(detection.id))
                .collect(Collectors.toList());
    }

    /**
     * <p>Converts a list of AprilTag detections to field-based distances.</p>
     * <p>The vector represents the translational offset from the tag, with the X and Y axes representing the forward and strafe offset, respectively.</p>
     *
     * @param detections A list of AprilTag detections to calculate the vectors of
     * @return A list of Vector2d objects containing the offsets of each tag
     */
    public List<Vector2d> getDetectionVectors(List<AprilTagDetection> detections) {
        return detections.stream()
                .map(tag -> {
                    // TODO: Verify whether the yaw or its negative should be used
                    double x = tag.ftcPose.range * Math.cos(tag.ftcPose.bearing);
                    double y = tag.ftcPose.range * Math.sin(tag.ftcPose.bearing);
                    return new Vector2d(x, y).rotated(-tag.ftcPose.yaw);
                }).collect(Collectors.toList());
    }

    /**
     * <p>Converts the current AprilTag detections to field-based distances.</p>
     * <p>The vector represents the translational offset from the tag, with the X and Y axes representing the forward and strafe offset, respectively.</p>
     *
     * @return A list of Vector2d objects containing the offsets of each tag
     */
    public List<Vector2d> getDetectionVectors() {
        return this.getDetectionVectors(this.getFilteredDetections());
    }

    /**
     * <p>Converts a list of AprilTag detections to Pose2d objects.</p>
     * <p>The X and Y axes of a pose constitute the camera's forward and strafe offsets, while the heading represents the yaw.</p>
     *
     * @param detections A list of AprilTag detections to calculate the poses of
     * @return A list of Pose2d objects containing the pose relative to each tag
     */
    public List<Pose2d> getDetectionPoses(List<AprilTagDetection> detections) {
        return detections.stream()
                .map(tag -> {
                    double x = tag.ftcPose.range * Math.cos(tag.ftcPose.bearing);
                    double y = tag.ftcPose.range * Math.sin(tag.ftcPose.bearing);
                    return new Pose2d(new Vector2d(x, y).rotated(-tag.ftcPose.yaw), tag.ftcPose.yaw);
                }).collect(Collectors.toList());
    }

    /**
     * <p>Converts the current AprilTag detections to Pose2d objects.</p>
     * <p>The X and Y axes of a pose constitute the camera's forward and strafe offsets, while the heading represents the yaw.</p>
     *
     * @return A list of Pose2d objects containing the pose relative to each tag
     */
    public List<Pose2d> getDetectionPoses() {
        return this.getDetectionPoses(this.getFilteredDetections());
    }

    public void shutdown() {
        if (portal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED)
            return;

        portal.close();
    }
}