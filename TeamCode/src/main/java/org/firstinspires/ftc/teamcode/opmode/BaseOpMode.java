package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.BulkReadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.CachingCRServo;
import org.firstinspires.ftc.teamcode.util.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;

public class BaseOpMode extends CommandOpMode {
    protected DcMotorEx leftSlideMotor, rightSlideMotor;
    protected IntakeSubsystem intake;
    protected DropSubsystem drop;
    protected BulkReadSubsystem bulkRead;
    protected GamepadEx driver1;
    protected GamepadEx driver2;
    protected TriggerGamepadEx t1;
    protected TriggerGamepadEx t2;
    protected CRServo axon;
    protected Motor fL, fR, bL, bR;
    protected Servo lS, rS, t;
    protected String[] REDLABEL = { "RedProp" };
    protected String[] BLUELABEL = { "BlueProp" };

    @Override
    public void initialize() {
        telemetry.update();
        initHardware();
        cacheHardware();
        setupHardware();

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        t1 = new TriggerGamepadEx(gamepad1, driver1);
        t2 = new TriggerGamepadEx(gamepad2, driver2);

        // Common subsystems go here (for auto and tele)
        intake = new IntakeSubsystem(axon);
        drop = new DropSubsystem(leftSlideMotor, rightSlideMotor, lS, rS, t);
        bulkRead = new BulkReadSubsystem(hardwareMap);

        telemetry.addData("Status", "BaseOpMode Initialized");
        telemetry.update();
    }

    protected void initHardware() {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlide");

        lS = hardwareMap.get(Servo.class, "leftServo");
        rS = hardwareMap.get(Servo.class, "rightServo");
        t = hardwareMap.get(Servo.class, "tray");

        axon = hardwareMap.get(CRServo.class, "axon");
    }

    protected void cacheHardware() {
        leftSlideMotor = new CachingDcMotorEX(leftSlideMotor);
        rightSlideMotor = new CachingDcMotorEX(rightSlideMotor);

        lS = new CachingServo(lS);
        rS = new CachingServo(rS);
        t = new CachingServo(t);

        axon = new CachingCRServo(axon);

        fL = new CachingMotor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        fR = new CachingMotor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        bL = new CachingMotor(hardwareMap, "leftRear", Motor.GoBILDA.RPM_312);
        bR = new CachingMotor(hardwareMap, "rightRear", Motor.GoBILDA.RPM_312);

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
}
