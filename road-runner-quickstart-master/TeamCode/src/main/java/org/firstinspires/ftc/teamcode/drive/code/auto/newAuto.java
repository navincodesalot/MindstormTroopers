//package org.firstinspires.ftc.teamcode.drive.code.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.code.util.autoDriver;
//import org.firstinspires.ftc.teamcode.drive.code.util.returns.returns;
//
//public class newAuto extends LinearOpMode {
//    Pose2d leftBlueStartPose = new Pose2d(35, 61, Math.toRadians(-90));
//    Pose2d rightBlueStartPose = new Pose2d(returns.returnX(35), 61, Math.toRadians(-90));
//
//    Pose2d leftRedStartPose = new Pose2d(35, returns.returnY(61), Math.toRadians(-270));
//    Pose2d rightRedStartPose = new Pose2d(returns.returnX(35), returns.returnY(61), Math.toRadians(-270));
//    autoDriver auto = new autoDriver(hardwareMap, leftBlueStartPose, 1, 1, 1, telemetry);
//
//    public void runOpMode() {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//    }
//}
