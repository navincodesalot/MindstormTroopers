package org.firstinspires.ftc.teamcode.opmode.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

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
        drive.setDefaultCommand(drive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, imu::getHeading)); // by default we're in field centric, we can add slowed modes as well

        drop.liftServo();

        tad("Status", "OpMode Initialized");
        telemetry.update();

        //Keybinds
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
        gb1(DPAD_LEFT).whenActive(
                drop.slideIdle()
        );
        gb1(A).whenActive( // incase
                drop.pickupPixel()
        );
        gb1(Y).whenActive( // incase
                drop.liftServo()
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
