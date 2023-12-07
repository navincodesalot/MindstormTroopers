//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
//
//public class LiftSlide extends SequentialCommandGroup {
//    public LiftSlide (SlideSubsystem drop) {
//        addCommands(
//                new InstantCommand(drop::pickupPixel),
//                new WaitCommand(200),
//                new InstantCommand(drop::slideLift)
//        );
//
//        addRequirements(drop);
//    }
//}
