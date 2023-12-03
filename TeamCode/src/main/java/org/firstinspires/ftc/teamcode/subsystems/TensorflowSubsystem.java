package org.firstinspires.ftc.teamcode.subsystems;

import android.os.Environment;
import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Comparator;
import java.util.List;

public class TensorflowSubsystem extends SubsystemBase {
    private final TfodProcessor tensorflowProcessor;
    public VisionPortal portal;

    public TensorflowSubsystem(VisionPortal portal, TfodProcessor processor) {
        tensorflowProcessor = processor;
        this.portal = portal;
    }

    public TensorflowSubsystem(HardwareMap hardwareMap, String cameraName, String modelName, String... labels) {
        tensorflowProcessor = new TfodProcessor.Builder()
                .setIsModelTensorFlow2(true)
                .setModelAspectRatio(4.0 / 3.0)
                .setModelInputSize(640)
                .setModelFileName(Environment.getExternalStorageDirectory().getPath() + "/FIRST/tflitemodels/" + modelName)
                .setModelLabels(labels)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true) // todo: for now
                .addProcessor(tensorflowProcessor)
                .setAutoStopLiveView(true)
                .build();
    }

    public void shutdown() {
        tensorflowProcessor.shutdown();
        portal.close();
    }

    public void setMinConfidence(double confidence) {
        tensorflowProcessor.setMinResultConfidence((float) confidence);
    }

    public List<Recognition> getDetections() {
        return tensorflowProcessor.getRecognitions();
    }

    public Recognition getBestDetection() {
        List<Recognition> detections = tensorflowProcessor.getRecognitions();
        return detections.stream()
                .max(Comparator.comparingDouble(Recognition::getConfidence))
                .orElse(null);
    }
}