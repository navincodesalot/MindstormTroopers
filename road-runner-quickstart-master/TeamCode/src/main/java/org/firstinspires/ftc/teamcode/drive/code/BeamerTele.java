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
        telemetry.addData("Arm Current", arm.getCurrentPosition());
        telemetry.addData("Slide Current", slide.getCurrentPosition());
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
        front_right.setDirection(DcMotorEx.Direction.FORWARD);
        back_right.setDirection(DcMotorEx.Direction.FORWARD);
        front_left.setDirection(DcMotorEx.Direction.REVERSE);
        back_left.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
            functionController();
        }
    }

    private void functionController() {
        if (gamepad2.b) { //close claw
            claw.setPosition(0.3);
        }

        if (gamepad2.x) { //open claw
            claw.setPosition(1);
        }
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            arm.setPower(returnArmPower(arm.getCurrentPosition(), -130)); //grab cone
        }
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            arm.setPower(returnArmPower(arm.getCurrentPosition(), -100)); //lift cone slightly
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            arm.setPower(returnArmPower(arm.getCurrentPosition(), 20)); //1st post todo maybe works
        }
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            arm.setPower(returnArmPower(arm.getCurrentPosition(), 20)); //put into bucket
        }
        if(gamepad2.y){
            bclaw.setPosition(0.92);
        }
        if(gamepad2.a){
            bclaw.setPosition(0);
        }
    }

    public double returnSlidePower(double state, double target) {
        double p = 0.07, i = 0.4, d = 0.001, f = 0.04;
        PIDController controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        double pid = controller.calculate(state, target);
        double power = pid + f;
        return power;
    }

    public double returnArmPower(double state, double target) {
        double p = 0.0055, i = 0.045, d = 0.00035, f = 0.000000001;
        double divide = 1.1;
        PIDController controller = new PIDController(p, i, d);
        final double ticks_in_degrees = 537.7 / 360.0; // for 360 degree rotation

        controller.setPID(p, i, d);
        double pid = controller.calculate(state, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double power = pid + ff;
        power /= divide;

        telemetry.addData("Pos", state);
        telemetry.addData("Target", target);
        telemetry.update();
        return power;
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