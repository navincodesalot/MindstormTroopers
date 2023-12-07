package org.firstinspires.ftc.teamcode.opmode.tele;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.DriveFieldCommand;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.function.DoubleSupplier;

@TeleOp(name = "CmdTele Test")
public class CmdOpMode extends BaseOpMode {
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        super.initialize();
        // Set Default Commands for each op mode (more intuitive)
        register(intake, drop, drive); // runs the peridoics? (idk)

        rrDrive.setPoseEstimate(PoseStorage.currentPose); // grab pose from auto

        DoubleSupplier imuHead = (() -> (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate);

        intake.setDefaultCommand(new RunCommand(intake::stop, intake)); // pass intake subsystem as the requirements (only for run commands??? i think)
        drive.setDefaultCommand(new DriveFieldCommand( // todo: do we need to schedule() it?
                drive,
                drop,
                gamepadEx1::getLeftX,
                gamepadEx1::getLeftY,
                gamepadEx1::getRightX,
                imuHead,
                true
        ));

        tad("Status", "OpMode Initialized");
        tad("imu", imuHead);
        telemetry.update();

        // Keybinds
        gb1(LEFT_TRIGGER).whileActiveContinuous(
                () -> schedule(new RunCommand(
                        () -> drive.slowMode(
                                gamepadEx1::getLeftX,
                                gamepadEx1::getLeftY,
                                gamepadEx1::getRightX
                        )
                )
        ));
        gb1(LEFT_BUMPER).whileHeld(
                () -> schedule(new InstantCommand(intake::grab)) //todo: check with kookys, if this doesn't work
        );
        gb1(RIGHT_BUMPER).whileHeld(
                () -> schedule(new InstantCommand(intake::push))
        );
        gb1(DPAD_UP).whenActive(
                () -> schedule(new InstantCommand(drop::slideLift))
        );
        gb1(DPAD_DOWN).whenActive(
                () -> schedule(new InstantCommand(drop::slideIdle))
        );
        gb1(DPAD_LEFT).whenActive(
                () -> schedule(new InstantCommand(drop::slideMiddle))
        );
        gb1(X).whenPressed(
                () -> schedule(new InstantCommand(drop::dropLeftPixel))
        );
        gb1(B).whenPressed(
                () -> schedule(new InstantCommand(drop::dropRightPixel))
        );

        //todo make a drop for both?

        // Manual commands
        gb1(A).whenPressed(
                () -> schedule(new InstantCommand(drop::pickupPixel))
        );
        gb1(Y).whenPressed(
                () -> schedule(new InstantCommand(drop::liftServo))
        );

        // Backup commands
        gb2(A).whenPressed(
                () -> schedule(new InstantCommand(drop::pickupPixel))
        );
        gb2(Y).whenPressed(
                () -> schedule(new InstantCommand(drop::liftServo))
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
