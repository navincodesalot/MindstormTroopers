package org.firstinspires.ftc.teamcode.drive.code.util.detection;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.code.util.detection.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AprilTagsUtil {

    Telemetry telemetry;
    private OpenCvWebcam webcam;
    private AprilTagDetectionPipeline pipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


    public AprilTagsUtil( HardwareMap hardwareMap, String webcamName, Telemetry telemetry ) {
        this.telemetry = telemetry;
        setup( hardwareMap, webcamName );
    }

    public void setup( HardwareMap hardwareMap, String webcamName ) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources( ).getIdentifier( "Webcam 1", "id", hardwareMap.appContext.getPackageName( ) );
        webcam = OpenCvCameraFactory.getInstance( ).createWebcam( hardwareMap.get( WebcamName.class, webcamName ), cameraMonitorViewId );
        pipeline = new AprilTagDetectionPipeline( tagsize, fx, fy, cx, cy, telemetry );
        webcam.setPipeline( pipeline );
    }

    public void init( ) {
        openCameraDevice( );
    }

    public void setTimeoutTime( int milliseconds ) {
        // Timeout for obtaining permission is configurable. Set before opening.
        webcam.setMillisecondsPermissionTimeout( milliseconds );
    }

    public void openCameraDevice( ) {

        webcam.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener( ) {
            @Override
            public void onOpened( ) {
                webcam.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT );
            }

            @Override
            public void onError( int errorCode ) {
                //This will be called if the camera could not be opened
                Log.e( "CAMERA_DEVICE", "Camera could not be opened. Error code: " + errorCode );
            }
        } );
    }

    public AprilTagDetectionPipeline.SignalPosition getSignalPosition( ) {
        return pipeline.getSignalPosition( );
    }

    public void stopCamera( ) {
        webcam.stopStreaming( );
    }
}