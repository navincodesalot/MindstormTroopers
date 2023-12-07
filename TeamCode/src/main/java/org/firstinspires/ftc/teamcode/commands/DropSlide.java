//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
//
//public class DropSlide extends SequentialCommandGroup {
//    public DropSlide (SlideSubsystem drop) {
//        addCommands(
//                new InstantCommand(drop::pickupPixel),
//                new WaitCommand(400),
//                new InstantCommand(drop::slideIdle)
//        );
//
//        addRequirements(drop);
//    }
//}
