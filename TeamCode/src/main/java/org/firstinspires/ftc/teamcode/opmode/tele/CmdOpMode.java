package org.firstinspires.ftc.teamcode.opmode.tele;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "CmdTele Test")
public class CmdOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        // Set Default Commands for each op mode (more intuitive)
        register(intake, slide, drive); // runs the peridoics (idk)
        intake.setDefaultCommand(intake.stop());
        drive.setDefaultCommand(drive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, imu::getHeading)); // by default we're in field centric, we can add slowed modes as well

        tad("Status", "OpMode Initialized");
        //Keybinds
        gb1(LEFT_BUMPER).whileHeld(
                intake.grab()
        );
        gb1(RIGHT_BUMPER).whileHeld(
                intake.push()
        );
        gb1(DPAD_UP).whenActive(
                slide.lift()
        );
        gb1(DPAD_LEFT).whenActive(
                slide.idle()
        );
        gb1(DPAD_DOWN).whenActive(
                slide.middle()
        );
    }

    @Override
    public void run() {
        super.run();
        tad("pos", leftSlideMotor.getCurrentPosition());
        // rrDrive.update() would go here
    }
}
