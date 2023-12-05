package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystems.TensorflowSubsystem;

@Autonomous
public class BlueAutoTest extends BaseOpMode {
    private PropLocations location;

    @Override
    public void initialize() {
        super.initialize();
        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "blueprop.tflite", LABELS);

        //visionPortal.stopLiveView(); // todo: when in comp
        tensorflow.setMinConfidence(0.75);

        while (opModeInInit()) {
            Recognition bestDetection = tensorflow.getBestDetection();
            location = PropLocations.MIDDLE;

            if (bestDetection != null) {
                double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2;

                if (x < 150) {
                    location = PropLocations.LEFT;
                } else if (x > 150 && x < 440) {
                    location = PropLocations.MIDDLE;
                } else {
                    location = PropLocations.RIGHT;
                }
            }

            telemetry.addData("FPS", tensorflow.portal.getFps()); //todo remove tele except loco
            telemetry.addData("Current Location", location.toString());
            telemetry.addData("Confidence", String.format("%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();
        }

        imu.reset();
        // rrDrive.setPoseEstimate(); todo

        waitForStart();

        schedule(new SequentialCommandGroup( // todo (look at powercube's auto)
                new InstantCommand(tensorflow::shutdown)
        ));
    }
    private enum PropLocations {
        LEFT,
        MIDDLE,
        RIGHT
    }
}
