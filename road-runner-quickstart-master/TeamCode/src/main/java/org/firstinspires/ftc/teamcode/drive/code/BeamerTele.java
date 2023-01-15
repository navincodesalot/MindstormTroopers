package org.firstinspires.ftc.teamcode.drive.code;

import android.transition.Slide;

import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.arcrobotics.ftclib.controller.PIDController;


@TeleOp
public class BeamerTele extends LinearOpMode {
    private BNO055IMU IMU;
    private DcMotorEx front_right;
    private DcMotorEx back_right;
    private DcMotorEx front_left;
    private DcMotorEx back_left;
    private DcMotorEx arm;
    private DcMotorEx slide;
    private Servo claw;
    private Servo bclaw;

    int Fast;

    float Roll;

    int Angle;
    List<Recognition> recognitions;
    int Time;
    int DetLocat;
    int Power;


    /**.0
     * Describe this function...
     */
    private void InitIMU() {
        BNO055IMU.Parameters imuParameters;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        IMU.initialize(imuParameters);
    }

    /**     * Describe this function...
     */
    private void Telemetry() {
        telemetry.addData("BackRightPos", back_right.getCurrentPosition());
        telemetry.addData("BackLeftPos", back_left.getCurrentPosition());
        telemetry.addData("imu", IMU.getAngularOrientation());
        telemetry.addData("Arm Current", arm.getCurrentPosition());
//        telemetry.addData("Lift 1", liftMotor1.getCurrentPosition());
        telemetry.addData("Slide Current", slide.getCurrentPosition());
        telemetry.addData("Yaw", IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.update();
    }
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        PhotonCore.enable();
        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        back_right = hardwareMap.get(DcMotorEx.class, "back_right");
        front_left = hardwareMap.get(DcMotorEx.class, "front_left");
        back_left = hardwareMap.get(DcMotorEx.class, "back_left");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        claw = hardwareMap.get(Servo.class, "claw");
        bclaw = hardwareMap.get(Servo.class, "bclaw");
        InitIMU();
        front_right.setDirection(DcMotorEx.Direction.FORWARD);
        back_right.setDirection(DcMotorEx.Direction.FORWARD);
        front_left.setDirection(DcMotorEx.Direction.REVERSE);
        back_left.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //        ArmMotor.setDirection(DcMotorEx.Direction.FORWARD);

        Fast = 1;
        waitForStart();
        arm.setTargetPosition(0);
        slide.setTargetPosition(0);
//        claw.setPosition(0.3); //close claw on init
        //Higher number is further down and vice versa
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            Telemetry();
            Movement();
            clawController();
        }
    }

    private void clawController() {
        if (gamepad2.b) { //close claw
            claw.setPosition(0.3);
        }

        if (gamepad2.x) { //open claw
            claw.setPosition(1);
        }
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            arm.setPower(returnPower(arm.getCurrentPosition(), 130)); //grab cone
        }
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            arm.setPower(returnPower(arm.getCurrentPosition(), 100)); //lift cone slightly
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            arm.setPower(returnPower(arm.getCurrentPosition(), 20)); //1st post todo maybe works
        }
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            arm.setPower(returnPower(arm.getCurrentPosition(), -25)); //put into bucket
        }
    }
    public double p = 0.005, i = 0.05, d = 0.0009, f = -0.008;
    public PIDController controller = new PIDController(p, i, d);

    public double returnPower(double state, double target) {
        final double ticks_in_degrees = 537.7 / 360.0; // for 360 degree rotation

        controller.setPID(p, i, d);
        double pid = controller.calculate(state, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;

        telemetry.addData("Pos", state);
        telemetry.addData("Target", target);
        telemetry.update();
        return power;
    }

    private void armController() {
//        //Higher number is further down and vice versa FOR claw
        if (gamepad2.dpad_up) {
            arm.setTargetPosition(136); //grab cone from bottom
        }
        if (gamepad2.dpad_right) {
            arm.setTargetPosition(-20); //put into bucket
        }
        if (gamepad2.dpad_down) {
            arm.setTargetPosition(115); //lift cone slightly
        }
        if (gamepad2.dpad_left) {
            arm.setTargetPosition(23); //1st post
        }
        //Need to make this function or pidf loop for slide as well
        if (gamepad2.y) {
            arm.setTargetPosition(arm.getTargetPosition() - 25);
        }
        if (gamepad2.a) {
            arm.setTargetPosition(arm.getTargetPosition() + 25);
        }
        if (arm.getCurrentPosition() < arm.getTargetPosition() - 1) {
            arm.setVelocity(-150);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        } else if (arm.getCurrentPosition() > arm.getTargetPosition() + 1) {
            arm.setVelocity(150);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
   }

   private void coneLoop() {
        boolean run = false;
        int targetPos = 0;
        if (gamepad1.y) {
            run = true;
            if (gamepad1.y) {
                targetPos = slide.getCurrentPosition();
                run = false;
            }
        }
        if (gamepad1.a && targetPos != 0) {
            // drop claw
            // close claw
            //pickup cone
            // run to slide
            slide.setTargetPosition(targetPos);
        }
   }
//    private void extendLift() {
////        //Higher number is further down and vice versa FOR DCLAW
//
//        if (gamepad2.x) {
//            claw.setPosition(0);
//        }
//        if (gamepad2.b) {
//            claw.setPosition(1);
//        }
//
//        if (gamepad2.right_bumper){
//            dclaw.setPosition(0);
//        }
//
//        if (gamepad2.left_bumper){
//            dclaw.setPosition(1);
//        }
//
//        if(gamepad2.right_stick_button){
//            arm.setPosition(0.5);
//        }
//
//        if(gamepad2.left_stick_button){
//            arm.setPosition(1);
//        }
//
//
//        if (gamepad2.dpad_up) {
//            liftMotor1.setTargetPosition(-1699);
//            liftMotor2.setTargetPosition(-1746);
////            arm.setPosition(0.5);
////            dclaw.setPosition(0.5);
//        }
//        if (gamepad2.dpad_right) {
//            liftMotor2.setTargetPosition(-1450);
//            liftMotor1.setTargetPosition(-1501);
////            arm.setPosition(0);
////            dclaw.setPosition(0.5);
//        }
//        if (gamepad2.dpad_down) {
//            liftMotor1.setTargetPosition(0);
//            liftMotor2.setTargetPosition(0);
////            arm.setPosition(0);
////            dclaw.setPosition(0);
//        }
//        if (gamepad2.y) {
//            liftMotor1.setTargetPosition(liftMotor1.getTargetPosition() - 25);
//            liftMotor2.setTargetPosition(liftMotor2.getTargetPosition() - 25);
//        }
//        if (gamepad2.a) {
//            liftMotor1.setTargetPosition(liftMotor1.getTargetPosition() + 25);
//            liftMotor2.setTargetPosition(liftMotor2.getTargetPosition() + 25);
//        }
//        if (liftMotor1.getCurrentPosition() < liftMotor1.getTargetPosition() - 50 && liftMotor2.getCurrentPosition() < liftMotor2.getTargetPosition() - 50) {
//            liftMotor1.setVelocity(-2000);
//            liftMotor2.setVelocity(-2000);
//            liftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        } else if (liftMotor1.getCurrentPosition() > liftMotor1.getTargetPosition() + 50 && liftMotor2.getCurrentPosition() > liftMotor2.getTargetPosition() + 50) {
//            liftMotor1.setVelocity(1300);
//            liftMotor2.setVelocity(1300);
//            liftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        }
//    }
//    private void extendedArm() {
//        if(gamepad2.b) {
//            Claw.setPosition(1);
//        }
//        if(gamepad2.x) {
//            Claw.setPosition(0);
//        }
//        if (gamepad2.right_bumper && StringMotor.getCurrentPosition() <= 250 && StringMotor.getCurrentPosition() >= -2100) {
//            StringMotor.setPower(-0.85);
//        } else if (gamepad2.left_bumper && StringMotor.getCurrentPosition() <= 250 && StringMotor.getCurrentPosition() >= -2100) {
//            StringMotor.setPower(0.23);
//        } else {
//            StringMotor.setPower(0);
//        }
//    }
//
//    private void Arm() {
//        //Higher number is further down and vice versa FOR DCLAW
//        if (gamepad2.y) {
//            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + 25);
//        }
//        if (gamepad2.a) {
//            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() - 25);
//        }
//        if (gamepad2.dpad_up) {
//            ArmMotor.setTargetPosition(1125);
//        }
//        if (gamepad2.dpad_right) {
//            ArmMotor.setTargetPosition(1080);
//        }
//        if (gamepad2.dpad_down) {
//            ArmMotor.setTargetPosition(10);
//        }
//
//        if (ArmMotor.getCurrentPosition() < ArmMotor.getTargetPosition() - 50) {
//            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//            try {
//                ((DcMotorEx) ArmMotor).setVelocity(-1000);
//                Thread.sleep(600);
//            } catch (InterruptedException e) {}
//            if(ArmMotor.getTargetPosition() == 1125 || ArmMotor.getTargetPosition() == 1080) {
//                DClaw.setPosition(0.9);
//            }
//        } else if (ArmMotor.getCurrentPosition() > ArmMotor.getTargetPosition() + 50) {
//            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            try {
//                ((DcMotorEx) ArmMotor).setVelocity(500);
//                Thread.sleep(600);
//            } catch (InterruptedException e){}
//            if(ArmMotor.getTargetPosition() == 10) {
//                DClaw.setPosition(0.71);
//            }
//        }
//    }
//
//    private void IMUTurnLeft() {
//        front_right.setDirection(DcMotorEx.Direction.REVERSE);
//        back_right.setDirection(DcMotorEx.Direction.REVERSE);
//        front_left.setDirection(DcMotorEx.Direction.REVERSE);
//        back_left.setDirection(DcMotorEx.Direction.REVERSE);
//        front_left.setPower(0.55);
//        back_left.setPower(0.55);
//        front_right.setPower(0.55);
//        back_right.setPower(0.55);
//    }
//
//    private void StopRobot() {
//        front_left.setPower(0);
//        back_left.setPower(0);
//        front_right.setPower(0);
//        back_right.setPower(0);
//    }
//
//    private void RotateLeftIMU() {
//        Roll = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//        while (!(Roll > Angle || isStopRequested())) {
//            Roll = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//            IMUTurnLeft();
//            telemetry.addData("Yaw", Roll);
//            telemetry.update();
//        }
//        StopRobot();
//    }
//
//    private void turnLeft180() {
//        if (gamepad1.right_bumper) {
//            Roll = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//            if(Roll < 0){
//                while ((Roll < (Roll+180) || isStopRequested())) {
//                    Roll = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
//                    IMUTurnLeft();
//                    telemetry.addData("Roll", Roll);
//                    telemetry.update();
//                }
//            }
//            StopRobot();
//        }
//    }

    private void slideCorrect() {
        if (gamepad1.x) {
            slide.setTargetPosition(-315); //second cone
        }
        if (gamepad1.b) {
            slide.setTargetPosition(-600); //3rd cone
        }
        if (gamepad1.y) {
            slide.setTargetPosition(slide.getTargetPosition() - 25);
        }
        if (gamepad1.a) {
            slide.setTargetPosition(slide.getTargetPosition() + 25);
        }

        if (slide.getCurrentPosition() < slide.getTargetPosition() - 25){
            slide.setPower(0.8);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        } else if (slide.getCurrentPosition() > slide.getTargetPosition() + 25) {
            slide.setVelocity(-0.8);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }

    private void Movement() {
        int my_0;
        int my_0New;
        float LeftStickY;
        float LeftStickX;
        float RightStickX;
        boolean ReturnVar;
        float LY;
        float LX;
        float RX;

        my_0 = 0;
        my_0New = 0;
        LeftStickY = gamepad1.left_stick_y;
        LeftStickX = gamepad1.left_stick_x;
        RightStickX = gamepad1.right_stick_x;
        ReturnVar = false;
        if (gamepad1.y) {
            Fast = 2;
        } else if (gamepad1.b) {
            Fast = 1;
        } else if (gamepad1.a) {
            Fast = 0;
        }
        if (Fast == 2) {
            if ((LeftStickY != my_0|| LeftStickX != my_0|| RightStickX != my_0)) {
                LY = gamepad1.left_stick_y;
                LX = gamepad1.left_stick_x;
                RX = gamepad1.right_stick_x;
                front_right.setPower(((LY + LX + RX) / 0.95));
                back_right.setPower(((LY + (LX - RX)) / 0.95));
                front_left.setPower(((LY - (LX + RX)) / 0.95));
                back_left.setPower(((LY - (LX - RX)) / 0.95));
            } else {
                front_right.setPower(0);
                back_right.setPower(0);
                front_left.setPower(0);
                back_left.setPower(0);
            }
        } else if (Fast == 1) {
            if ((LeftStickY != my_0|| LeftStickX != my_0|| RightStickX != my_0)) {
                LY = gamepad1.left_stick_y;
                LX = gamepad1.left_stick_x;
                RX = gamepad1.right_stick_x;
                front_right.setPower(((LY + LX + RX) / 0.95) / 1.75);
                back_right.setPower(((LY + (LX - RX)) / 0.95) / 1.75);
                front_left.setPower(((LY - (LX + RX)) / 0.95) / 1.75);
                back_left.setPower(((LY - (LX - RX)) / 0.95) / 1.75);
            } else {
                front_right.setPower(0);
                back_right.setPower(0);
                front_left.setPower(0);
                back_left.setPower(0);
            }
        } else if (Fast == 0) {
            if ((LeftStickY != my_0|| LeftStickX != my_0|| RightStickX != my_0)){
                LY = gamepad1.left_stick_y;
                LX = gamepad1.left_stick_x;
                RX = gamepad1.right_stick_x;
                front_right.setPower(((LY + LX + RX) / 0.95) / 3.5);
                back_right.setPower(((LY + (LX - RX)) / 0.95) / 3.5);
                front_left.setPower(((LY - (LX + RX)) / 0.95) / 3.5);
                back_left.setPower(((LY - (LX - RX)) / 0.95) / 3.5);
            } else {
                front_right.setPower(0);
                back_right.setPower(0);
                front_left.setPower(0);
                back_left.setPower(0);
            }
        }
    }
}