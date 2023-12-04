package org.firstinspires.ftc.teamcode.opmode.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import org.firstinspires.ftc.teamcode.commands.DriveFieldCommand;
import org.firstinspires.ftc.teamcode.commands.DropSlide;
import org.firstinspires.ftc.teamcode.commands.IntakePixel;
import org.firstinspires.ftc.teamcode.commands.LiftSlide;
import org.firstinspires.ftc.teamcode.commands.PushPixel;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "CmdTele Test")
public class CmdOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        // Set Default Commands for each op mode (more intuitive)
        register(intake, drop, drive); // runs the peridoics? (idk)

        intake.setDefaultCommand(intake.stop());

        DriveFieldCommand DriveFieldCommand = new DriveFieldCommand(
                drive,
                drop,
                () -> gamepadEx1.getLeftX(),
                () -> gamepadEx1.getLeftY(),
                () -> gamepadEx1.getRightX(),
                imu::getHeading,
                true
        );

        drive.setDefaultCommand(DriveFieldCommand);

        drop.liftServo();

        tad("Status", "OpMode Initialized");
        telemetry.update();

        // Keybinds
        gb1(LEFT_TRIGGER).whileActiveContinuous(
                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX)
        );
        gb1(LEFT_BUMPER).whileHeld(
                new IntakePixel(intake, drop)
        );
        gb1(RIGHT_BUMPER).whileHeld(
                new PushPixel(intake, drop)
        );
        gb1(DPAD_UP).whenActive(
                new LiftSlide(drop)
        );
        gb1(DPAD_DOWN).whenActive(
                new DropSlide(drop)
        );
        gb1(X).whenActive(
                drop.dropLeftPixel()
        );
        gb1(B).whenActive(
                drop.dropRightPixel()
        );

        // Manual commands
        gb1(A).whenActive(
                drop.pickupPixel()
        );
        gb1(Y).whenActive(
                drop.liftServo()
        );
        gb1(DPAD_LEFT).whenActive(
                drop.slideIdle()
        );
        // Backup commands
        gb2(A).whenActive(
                drop.pickupPixel()
        );
        gb2(Y).whenActive(
                drop.liftServo()
        );
        gb2(DPAD_LEFT).whenActive(
                drop.slideIdle()
        );
    }

    @Override
    public void run() {
        super.run();
        tad("left slide pos", leftSlideMotor.getCurrentPosition());
        telemetry.update();
        // rrDrive.update() would go here
    }
}
