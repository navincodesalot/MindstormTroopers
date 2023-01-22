package org.firstinspires.ftc.teamcode.drive.code.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.code.util.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.code.util.pidf.armPIDF;
import org.firstinspires.ftc.teamcode.drive.code.util.pidf.slidePIDF;

@TeleOp
public class testTele extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        CYCLE
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    public DcMotorEx arm;
    public DcMotorEx slide;
    public Servo claw;
    public Servo bclaw;

    double clawClose = 0.3;
    int armTarget = 0;
    int slideTarget = 0;
    int armPickup = -100;
    boolean save = false;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        arm.setTargetPosition(0);
        slide.setTargetPosition(0);
        claw.setPosition(clawClose); //close claw on init

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            slide.setPower(slidePIDF.returnPower(slide.getCurrentPosition(), slideTarget));
            arm.setPower(armPIDF.returnPower(arm.getCurrentPosition(), armTarget));

            switch(currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * 0.6,
                                    -gamepad1.left_stick_x * 0.6,
                                    -gamepad1.right_stick_x * 0.6
                            )
                    );
                    drive.update();

                    if(gamepad1.y) { //drop
                        bclaw.setPosition(0.92);
                    }
                    if(gamepad1.a) { //pickup
                        bclaw.setPosition(0);
                    }
                    if (gamepad2.right_bumper) {
                        slideTarget = 2650;
                    }
                    if (gamepad2.left_bumper) {
                        slideTarget = 0;
                    }
                    if (gamepad2.b) { //close claw
                        claw.setPosition(clawClose);
                    }
                    if (gamepad2.x) { //open claw
                        claw.setPosition(1);
                    }
                    if(gamepad2.left_stick_button){
                        armTarget = -20;
                    }
                    if (gamepad2.a) {
                        armTarget = armPickup;
                    }
                    if (gamepad2.y) {
                        armTarget = 20;
                    }
                    if (gamepad1.x) {
                        save = true;
                        currentMode = Mode.CYCLE;
                    }
                    break;
                case CYCLE:
                    int targetPos = 0;
                    if (gamepad1.x && save) {
                        telemetry.addData("Saving", targetPos);
                        targetPos = slide.getCurrentPosition();
                        save = false;
                    }

                    if (gamepad1.b && targetPos != 0) {
                        slideTarget = 0; //reset to 0 in case
                        claw.setPosition(1); //open claw
                        armTarget = armPickup; //grab cone
                        claw.setPosition(clawClose); //close claw
                        armTarget = 20; //put into bucket
                        claw.setPosition(1); //open claw
                        armTarget = -20; //move arm
                        slideTarget = targetPos; // go to target
                        bclaw.setPosition(0.92); //drop cone
                        sleep(1000); //wait to drop
                        bclaw.setPosition(0); //reset
                        slideTarget = 0; //bring slide down
                    }

                    if(gamepad1.right_bumper) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
            }
        }
    }
}