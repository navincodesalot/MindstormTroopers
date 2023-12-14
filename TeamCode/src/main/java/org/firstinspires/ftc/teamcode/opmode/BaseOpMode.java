package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BulkReadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;

public class BaseOpMode extends CommandOpMode {
    protected DcMotorEx intakeMotor, leftSlideMotor, rightSlideMotor;
    protected IntakeSubsystem intake;
    protected DropSubsystem drop;
    protected MecanumDriveSubsystem drive;
    protected BulkReadSubsystem bulkRead;
    protected GamepadEx driver1;
    protected GamepadEx driver2;
    protected TriggerGamepadEx t1;
    protected TriggerGamepadEx t2;
    protected SampleMecanumDrive rrDrive;
    protected IMU imu;
    protected MotorEx fL, fR, bL, bR;
    protected Servo lS, rS;
    protected String[] LABELS = { // todo: when we train new model
            "left",
            "right"
    };
    private double loopTime = 0.0;

    @Override
    public void initialize() {
        telemetry.update();
        rrDrive = new SampleMecanumDrive(hardwareMap);

        initHardware();
        setupHardware();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        t1 = new TriggerGamepadEx(gamepad1, driver1);
        t2 = new TriggerGamepadEx(gamepad2, driver2);

        // Subsystems go here
        intake = new IntakeSubsystem(intakeMotor);
        drop = new DropSubsystem(leftSlideMotor, rightSlideMotor, lS, rS);
        drive = new MecanumDriveSubsystem(fL, fR, bL, bR, imu);
        bulkRead = new BulkReadSubsystem(hardwareMap);

        tad("Status", "BaseOpMode Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run(); // since we are overriding in opmodes, this will actually run it
        double loop = System.nanoTime();
        tad("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
    }

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

        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // telemetry add data = tad
    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    protected void tal(String caption) {
        telemetry.addLine(caption);
    }
}
