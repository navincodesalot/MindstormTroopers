package org.firstinspires.ftc.teamcode.opmode.tele;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name = "cooked ahh box")
public class CmdOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        super.initialize();
        register(drop, drive, bulkRead);
//        rrDrive.setPoseEstimate(PoseStorage.currentPose); // grab pose from auto

        // Set Default Commands for each op mode (more intuitive)
//        intake.setDefaultCommand(new RunCommand(intake::stop, intake));
        drive.setDefaultCommand(new RunCommand(() -> drive.fieldCentric(driver1::getLeftX, driver1::getLeftY, driver1::getRightX, () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), drive));
        tad("Status", "OpMode Initialized");
        // Keybinds
        t1.getGamepadTrigger(LEFT_TRIGGER).whileActiveContinuous(
                new RunCommand(() -> drive.slowMode(driver1::getLeftX, driver1::getLeftY, driver1::getRightX), drive)
        );
        // todo add keybind for brakes: (powerLimit)
        driver1.getGamepadButton(BACK).toggleWhenPressed(
                new RunCommand(() -> drive.robotCentric(driver1::getLeftX, driver1::getLeftY, driver1::getRightX), drive),
                new RunCommand(() -> drive.fieldCentric(driver1::getLeftX, driver1::getLeftY, driver1::getRightX, () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)), drive)
        );
//        driver1.getGamepadButton(LEFT_BUMPER).whileHeld(
//                new RunCommand(intake::grab, intake)
//        );
//        driver1.getGamepadButton(RIGHT_BUMPER).whileHeld(
//                new RunCommand(intake::push, intake)
//        );
        driver1.getGamepadButton(DPAD_UP).whenPressed(
                new InstantCommand(drop::slideLift, drop)
        );
        driver1.getGamepadButton(DPAD_DOWN).whenPressed(
                new InstantCommand(drop::slideIdle, drop)
        );
        driver1.getGamepadButton(DPAD_LEFT).whenPressed(
                new InstantCommand(drop::slideMiddle, drop)
        );
        driver1.getGamepadButton(X).whenPressed(
                new InstantCommand(drop::dropLeftPixel, drop)
        );
        driver1.getGamepadButton(B).whenPressed(
                new InstantCommand(drop::dropRightPixel, drop)
        );

        //todo make a drop for both?

        // Manual commands
        driver1.getGamepadButton(A).whenPressed(
                new InstantCommand(drop::pickupPixel, drop)
        );
        driver1.getGamepadButton(Y).whenPressed(
                new InstantCommand(drop::liftServo, drop)
        );

        // Backup commands
        driver2.getGamepadButton(A).whenPressed(
                new InstantCommand(drop::pickupPixel, drop)
        );
        driver2.getGamepadButton(Y).whenPressed(
                new InstantCommand(drop::liftServo, drop)
        );
    }

    @Override
    public void run() {
        super.run();
        rrDrive.update();
    }
}
