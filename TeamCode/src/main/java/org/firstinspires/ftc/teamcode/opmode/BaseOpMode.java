package org.firstinspires.ftc.teamcode.opmode;

import static com.google.blocks.ftcrobotcontroller.hardware.HardwareType.IMU;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;

public class BaseOpMode extends CommandOpMode {
    protected DcMotorEx intakeMotor, leftSlideMotor, rightSlideMotor;
    protected IntakeSubsystem intake;
    protected SlideSubsystem slide;
    protected MecanumDriveSubsystem drive;
    protected SampleMecanumDrive rrDrive;
    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    protected TriggerGamepadEx triggerGamepadEx1;
    protected TriggerGamepadEx triggerGamepadEx2;
    protected RevIMU imu;
    protected MotorEx fL, fR, bL, bR;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        tad("Status", "BaseOpMode Initializing");
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);

        initHardware();
        setupHardware();

        imu = new RevIMU(hardwareMap);
//        IMU.Parameters parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//                )
//        );
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";


//        imu.init(parameters); //todo: if we switch hub positioning change here

        drive = new MecanumDriveSubsystem(fL, fR, bL, bR, imu);
//        rrDrive = new SampleMecanumDrive(hardwareMap); todo: rr drive here if needed
//        rrDrive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(-90))); todo

        //Subsystems go here
        intake = new IntakeSubsystem(intakeMotor);
        slide = new SlideSubsystem(leftSlideMotor, rightSlideMotor);

        tad("Status", "BaseOpMode Initialized");
    }

    protected void initHardware() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlide");
        fL = new MotorEx(hardwareMap, "leftFront");
        fR = new MotorEx(hardwareMap, "rightFront");
        bL = new MotorEx(hardwareMap, "leftRear");
        bR = new MotorEx(hardwareMap, "rightRear");
    }

    protected void setupHardware() {
        fR.setInverted(true);
        bL.setInverted(true);
        //set modes and reset encoders here
        leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
        telemetry.update();
    }

    protected void tal(String caption) {
        telemetry.addLine(caption);
        telemetry.update();
    }
}
