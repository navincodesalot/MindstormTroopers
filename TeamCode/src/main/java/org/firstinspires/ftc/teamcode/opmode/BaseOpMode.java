package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;

public class BaseOpMode extends CommandOpMode {
    protected DcMotorEx intakeMotor, leftSlideMotor, rightSlideMotor;
    protected IntakeSubsystem intake;
    protected DropSubsystem drop;
    protected MecanumDriveSubsystem drive;
    protected SampleMecanumDrive rrDrive;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    protected TriggerGamepadEx triggerGamepadEx1;
    protected TriggerGamepadEx triggerGamepadEx2;
    protected IMU imu;
    protected MotorEx fL, fR, bL, bR;
    protected Servo lS, rS;
    protected String[] LABELS = { // todo: when we train new model
            "left",
            "right"
    };

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        tad("Status", "BaseOpMode Initializing");
        telemetry.update();
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);

        initHardware();
        setupHardware();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

//        rrDrive = new SampleMecanumDrive(hardwareMap); todo: rr drive here if needed
//        rrDrive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(-90))); todo

        // Subsystems go here
        intake = new IntakeSubsystem(intakeMotor);
        drop = new DropSubsystem(leftSlideMotor, rightSlideMotor, lS, rS);
        drive = new MecanumDriveSubsystem(fL, fR, bL, bR, imu);

//        initTfod(); // Init Camera

        tad("Status", "BaseOpMode Initialized");
        telemetry.update();
    }

//    protected void initTfod() {
//        tfod = new TfodProcessor.Builder()
//                .setModelFileName(TFOD_MODEL_FILE)
//                .setModelLabels(LABELS)
//                .setIsModelTensorFlow2(true)
//                .setIsModelQuantized(true)
//                .setModelInputSize(300)
//                .setModelAspectRatio(16.0 / 9.0)
//                .build();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        builder.setCameraResolution(new Size(640, 480));
//        builder.enableLiveView(true);
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        builder.setAutoStopLiveView(true);
//        builder.addProcessor(tfod);
//
//        visionPortal = builder.build();
//        tfod.setMinResultConfidence(0.75f);
//        visionPortal.setProcessorEnabled(tfod, true);
//    }

    protected void initHardware() {
        imu = hardwareMap.get(IMU.class, "imu");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlide");
        lS = hardwareMap.get(Servo.class, "leftServo");
        rS = hardwareMap.get(Servo.class, "rightServo");
        fL = new MotorEx(hardwareMap, "leftFront");
        fR = new MotorEx(hardwareMap, "rightFront");
        bL = new MotorEx(hardwareMap, "leftRear");
        bR = new MotorEx(hardwareMap, "rightRear");
    }

    protected void setupHardware() {
        fR.setInverted(true);
        bL.setInverted(true);
        lS.setDirection(Servo.Direction.FORWARD);
        rS.setDirection(Servo.Direction.REVERSE);
        //set modes and reset encoders here
        leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public GamepadButton gb1(GamepadKeys.Button button) {
        return gamepadEx1.getGamepadButton(button);
    }

    public GamepadButton gb2(GamepadKeys.Button button) {
        return gamepadEx2.getGamepadButton(button);
    }

    public GamepadTrigger gb1(GamepadKeys.Trigger trigger) {
        return triggerGamepadEx1.getGamepadTrigger(trigger);
    }

    public GamepadTrigger gb2(GamepadKeys.Trigger trigger) {
        return triggerGamepadEx1.getGamepadTrigger(trigger);
    }

    // telemetry add data = tad
    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    protected void tal(String caption) {
        telemetry.addLine(caption);
    }
}
