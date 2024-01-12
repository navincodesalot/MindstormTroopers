package org.firstinspires.ftc.teamcode.opmode.tele;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import org.firstinspires.ftc.teamcode.commands.DropSlide;
import org.firstinspires.ftc.teamcode.commands.LiftSlideHigh;
import org.firstinspires.ftc.teamcode.commands.LiftSlideLow;
import org.firstinspires.ftc.teamcode.commands.LiftSlideMed;
import org.firstinspires.ftc.teamcode.commands.PushOnePixel;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

//@Photon
@TeleOp(name = "cooked ahh tray")
public class CmdOpMode extends BaseOpMode {
    private IMU imu;
    private MecanumDriveSubsystem drive;
    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        imu = hardwareMap.get(IMU.class, "imu"); // init imu first for teleop (takes longer to init)
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR
                )
        ));

        super.initialize();

        drive = new MecanumDriveSubsystem(fL, fR, bL, bR, imu);
        register(drop, drive);

        // Set Default Commands for each op mode (more intuitive)
        intake.setDefaultCommand(new RunCommand(intake::stop, intake));

        //in init:
        drop.liftTray();

//        drive.setDefaultCommand(new RunCommand(() -> drive.fieldCentric(driver1::getLeftX, driver1::getLeftY, driver1::getRightX, () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), drive));
        drive.setDefaultCommand(new RunCommand(() -> drive.robotCentric(driver1::getLeftX, driver1::getLeftY, driver1::getRightX), drive));

        telemetry.addData("Status", "OpMode Initialized");

        // Keybinds
        t1.getGamepadTrigger(LEFT_TRIGGER).whileActiveContinuous(
                new RunCommand(() -> drive.slowMode(driver1::getLeftX, driver1::getLeftY, driver1::getRightX), drive)
        );
//        driver1.getGamepadButton(BACK).toggleWhenPressed(
//                new RunCommand(() -> drive.robotCentric(driver1::getLeftX, driver1::getLeftY, driver1::getRightX), drive),
//                new RunCommand(() -> drive.fieldCentric(driver1::getLeftX, driver1::getLeftY, driver1::getRightX, () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), drive)
//        );
        driver1.getGamepadButton(LEFT_BUMPER).whileHeld(
                new RunCommand(intake::push, intake)
        );
        driver1.getGamepadButton(RIGHT_BUMPER).whileHeld(
                new RunCommand(intake::grab, intake)
        );
        driver1.getGamepadButton(DPAD_UP).whenPressed(
                new LiftSlideHigh(drop)
        );
        driver1.getGamepadButton(DPAD_RIGHT).whenPressed(
                new LiftSlideMed(drop)
        );
        driver1.getGamepadButton(DPAD_DOWN).whenPressed(
                new DropSlide(drop)
        );
        driver1.getGamepadButton(DPAD_LEFT).whenPressed(
                new LiftSlideLow(drop)
        );
        driver1.getGamepadButton(X).whenPressed(
                new InstantCommand(drop::dropPixel, drop)
        );
        driver1.getGamepadButton(B).whenPressed(
                new PushOnePixel(intake)
        );
        driver1.getGamepadButton(Y).whenPressed(
                new InstantCommand(drop::liftTray, drop)
        );
        driver1.getGamepadButton(A).whenPressed(
                new InstantCommand(drop::pickupPixel, drop)
        );

        // Backup commands
        driver2.getGamepadButton(DPAD_UP).whenPressed(
                new LiftSlideHigh(drop)
        );
        driver2.getGamepadButton(DPAD_RIGHT).whenPressed(
                new LiftSlideMed(drop)
        );
        driver2.getGamepadButton(DPAD_DOWN).whenPressed(
                new DropSlide(drop)
        );
        driver2.getGamepadButton(DPAD_LEFT).whenPressed(
                new LiftSlideLow(drop)
        );
        driver2.getGamepadButton(X).whenPressed(
                new InstantCommand(drop::dropPixel, drop)
        );
        driver2.getGamepadButton(B).whenPressed(
                new InstantCommand(drop::dropPixel, drop)
        );
        driver2.getGamepadButton(A).whenPressed(
                new InstantCommand(drop::pickupPixel, drop)
        );
        driver2.getGamepadButton(Y).whenPressed(
                new InstantCommand(drop::liftTray, drop)
        );
    }

    @Override
    public void run() {
        super.run(); // since we are overriding in opmodes, this will actually run it
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        bulkRead.read();
    }
}
