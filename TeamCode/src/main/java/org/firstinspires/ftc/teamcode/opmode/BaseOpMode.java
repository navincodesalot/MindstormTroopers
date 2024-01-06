package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.BulkReadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;

public class BaseOpMode extends CommandOpMode {
    protected DcMotorEx leftSlideMotor, rightSlideMotor;
    protected IntakeSubsystem intake;
    protected VoltageSensor batteryVoltageSensor;
    protected DropSubsystem drop;
    protected BulkReadSubsystem bulkRead;
    protected GamepadEx driver1;
    protected GamepadEx driver2;
    protected TriggerGamepadEx t1;
    protected TriggerGamepadEx t2;
    protected CRServo axon;
    protected Motor fL, fR, bL, bR;
    protected Servo lS, rS, t;
    protected String[] REDLABEL = {
            "RedProp"
    };
    protected String[] BLUELABEL = {
            "BlueProp"
    };
    private double loopTime = 0.0;

    @Override
    public void initialize() {
        telemetry.update();
        initHardware();
        setupHardware();

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        t1 = new TriggerGamepadEx(gamepad1, driver1);
        t2 = new TriggerGamepadEx(gamepad2, driver2);

        // Common subsystems go here (for auto and tele)
        intake = new IntakeSubsystem(axon);
        drop = new DropSubsystem(leftSlideMotor, rightSlideMotor, lS, rS, t, batteryVoltageSensor);
        bulkRead = new BulkReadSubsystem(hardwareMap);

        tad("Status", "BaseOpMode Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); // since we are overriding in opmodes, this will actually run it
        double loop = System.nanoTime();
        tad("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
    }

    protected void initHardware() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlide");

        lS = hardwareMap.get(Servo.class, "leftServo");
        rS = hardwareMap.get(Servo.class, "rightServo");
        t = hardwareMap.get(Servo.class, "tray");

        axon = hardwareMap.get(CRServo.class, "axon");

        fL = new Motor(hardwareMap, "leftFront");
        fR = new Motor(hardwareMap, "rightFront");
        bL = new Motor(hardwareMap, "leftRear");
        bR = new Motor(hardwareMap, "rightRear");
    }

    protected void setupHardware() {
        fR.setInverted(true);
        bL.setInverted(true);

        lS.setDirection(Servo.Direction.FORWARD);
        rS.setDirection(Servo.Direction.REVERSE);
        t.setDirection(Servo.Direction.REVERSE);

        axon.setDirection(CRServo.Direction.FORWARD);

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

    // telemetry add data = tad
    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    protected void tal(String caption) {
        telemetry.addLine(caption);
    }
}
