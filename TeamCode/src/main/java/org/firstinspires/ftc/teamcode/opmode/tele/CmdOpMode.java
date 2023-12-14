package org.firstinspires.ftc.teamcode.opmode.tele;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveSlowCommand;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;

@TeleOp(name = "CmdTele Test")
public class CmdOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        super.initialize();
        register(intake, drop, drive, bulkRead);
        rrDrive.setPoseEstimate(PoseStorage.currentPose); // grab pose from auto

        // Set Default Commands for each op mode (more intuitive)
        intake.setDefaultCommand(new RunCommand(intake::stop, intake)); // pass intake subsystem as the requirements (only for run commands??? i think)

        // todo: fix drives (fieldCentric, slow, robotCentric)
        DriveCommand driveCommand = new DriveCommand(drive, rrDrive, driver1, rrDrive.getPoseEstimate());
        drive.setDefaultCommand(driveCommand);

        tad("Status", "OpMode Initialized");

        // Keybinds
        t1.getGamepadTrigger(LEFT_TRIGGER).whileActiveContinuous(
                new DriveSlowCommand(drive, rrDrive, driver1, rrDrive.getPoseEstimate())
        );
        driver1.getGamepadButton(LEFT_BUMPER).whileHeld(
                new RunCommand(intake::grab, intake) //todo: check with kookys, if this doesn't work
        );
        driver1.getGamepadButton(RIGHT_BUMPER).whileHeld(
                new RunCommand(intake::push, intake)
        );
        driver1.getGamepadButton(DPAD_UP).whenPressed(
                new InstantCommand(drop::slideLift)
        );
        driver1.getGamepadButton(DPAD_DOWN).whenPressed(
                new InstantCommand(drop::slideIdle)
        );
        driver1.getGamepadButton(DPAD_LEFT).whenPressed(
                new InstantCommand(drop::slideMiddle)
        );
        driver1.getGamepadButton(X).whenPressed(
                new InstantCommand(drop::dropLeftPixel)
        );
        driver1.getGamepadButton(B).whenPressed(
                new InstantCommand(drop::dropRightPixel)
        );

        //todo make a drop for both?

        // Manual commands
        driver1.getGamepadButton(A).whenPressed(
                new InstantCommand(drop::pickupPixel)
        );
        driver1.getGamepadButton(Y).whenPressed(
                new InstantCommand(drop::liftServo)
        );

        // Backup commands
        driver2.getGamepadButton(A).whenPressed(
                new InstantCommand(drop::pickupPixel)
        );
        driver2.getGamepadButton(Y).whenPressed(
                new InstantCommand(drop::liftServo)
        );
    }

    @Override
    public void run() {
        super.run();
        rrDrive.update();
    }
}
