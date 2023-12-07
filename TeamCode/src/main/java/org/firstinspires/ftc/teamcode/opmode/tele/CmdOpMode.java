//package org.firstinspires.ftc.teamcode.opmode.tele;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
//import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.commands.DriveFieldCommand;
//import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
//
//import java.util.function.DoubleSupplier;
//
//@TeleOp(name = "CmdTele Test")
//public class CmdOpMode extends BaseOpMode {
//    @Override
//    public void initialize() {
//        super.initialize();
//        // Set Default Commands for each op mode (more intuitive)
//        register(intake, slide, drive); // runs the peridoics? (idk)
//
//        intake.setDefaultCommand(intake.stop());
//
//        DriveFieldCommand DriveFieldCommand = new DriveFieldCommand(
//                drive,
//                slide,
//                () -> gamepadEx1.getLeftX(),
//                () -> gamepadEx1.getLeftY(),
//                () -> gamepadEx1.getRightX(),
//                () -> (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate,
//                true
//        );
//
//        DoubleSupplier imuHead = (() -> (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate);
//
//        drive.setDefaultCommand(drive.fieldCentric(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX, imuHead, true));
//        tad("Status", "OpMode Initialized");
//        tad("imu", imuHead);
//        telemetry.update();
//
//        // Keybinds
//        gb1(LEFT_TRIGGER).whileActiveContinuous(
//                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getLeftY, gamepadEx1::getRightX)
//        );
//        gb1(LEFT_BUMPER).whileHeld(
//                intake.grab()
//        );
//        gb1(RIGHT_BUMPER).whileHeld(
//                intake.push()
//        );
//        gb1(DPAD_UP).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(slide::pickupPixel),
//                        new WaitCommand(400),
//                        new InstantCommand(slide::slideIdle)
//                )
//        );
//        gb1(DPAD_DOWN).whenPressed(
//                slide.slideIdle()
//        );
//        gb1(X).whenPressed(
//                slide.dropLeftPixel()
//        );
//        gb1(B).whenPressed(
//                slide.dropRightPixel()
//        );
//
//        // Manual commands
//        gb1(A).whenActive(
//                slide.pickupPixel()
//        );
//        gb1(Y).whenActive(
//                slide.liftServo()
//        );
//        gb1(DPAD_LEFT).whenActive(
//                slide.slideIdle()
//        );
//        // Backup commands
//        gb2(A).whenActive(
//                slide.pickupPixel()
//        );
//        gb2(Y).whenActive(
//                slide.liftServo()
//        );
//        gb2(DPAD_LEFT).whenActive(
//                slide.slideIdle()
//        );
//        while (opModeInInit()) {
//            slide.liftServo();
//        }
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        tad("left slide pos", leftSlideMotor.getCurrentPosition());
//        telemetry.update();
//        // rrDrive.update() would go here
//    }
//}
