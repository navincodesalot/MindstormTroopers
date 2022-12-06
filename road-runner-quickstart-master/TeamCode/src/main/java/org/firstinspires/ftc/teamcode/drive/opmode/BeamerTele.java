package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@TeleOp(name = "BeamerTele (Blocks to Java)")
public class BeamerTele extends LinearOpMode {

    private BNO055IMU IMU;
    private DcMotor front_right;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor back_left;
    private DcMotor ArmMotor;
    private Servo Claw;
    private Servo DClaw;
    private DcMotor StringMotor;

    int Fast;

    float Roll;

    int Angle;
    List<Recognition> recognitions;
    int Time;
    int DetLocat;
    int Power;


    /**
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
        telemetry.addData("Arm Target", ArmMotor.getTargetPosition());
        telemetry.addData("Arm Current", ArmMotor.getCurrentPosition());
        telemetry.addData("String Motor Position: ", StringMotor.getCurrentPosition());
        telemetry.addData("Yaw", IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).thirdAngle);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        StringMotor = hardwareMap.get(DcMotor.class, "StringMotor");
        Claw = hardwareMap.get(Servo.class, "Claw");
        DClaw = hardwareMap.get(Servo.class, "DClaw");
        InitIMU();
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Fast = 1;
        DClaw.setPosition(0.71);
        waitForStart();
        ArmMotor.setTargetPosition(0);
        //Higher number is further down and vice versa
        while (opModeIsActive()) {
            Telemetry();
            Movement();
            Arm();
            extendedArm();
        }
    }

    private void extendedArm() {
        if(gamepad2.b) {
            Claw.setPosition(1);
        }
        if(gamepad2.x) {
            Claw.setPosition(0);
        }
        if (gamepad2.right_bumper && StringMotor.getCurrentPosition() <= 250 && StringMotor.getCurrentPosition() >= -2100) {
            StringMotor.setPower(-0.85);
        } else if (gamepad2.left_bumper && StringMotor.getCurrentPosition() <= 250 && StringMotor.getCurrentPosition() >= -2100) {
            StringMotor.setPower(0.23);
        } else {
            StringMotor.setPower(0);
        }
    }

    private void Arm() {
        //Higher number is further down and vice versa FOR DCLAW
        if (gamepad2.y) {
            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + 25);
        }
        if (gamepad2.a) {
            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() - 25);
        }
        if (gamepad2.dpad_up) {
            ArmMotor.setTargetPosition(1125);
        }
        if (gamepad2.dpad_right) {
            ArmMotor.setTargetPosition(1080);
        }
        if (gamepad2.dpad_down) {
            ArmMotor.setTargetPosition(10);
        }

        if (ArmMotor.getCurrentPosition() < ArmMotor.getTargetPosition() - 50) {
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            try {
                ((DcMotorEx) ArmMotor).setVelocity(-1000);
                Thread.sleep(600);
            } catch (InterruptedException e) {}
            if(ArmMotor.getTargetPosition() == 1125 || ArmMotor.getTargetPosition() == 1080) {
                DClaw.setPosition(0.9);
            }
        } else if (ArmMotor.getCurrentPosition() > ArmMotor.getTargetPosition() + 50) {
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            try {
                ((DcMotorEx) ArmMotor).setVelocity(500);
                Thread.sleep(600);
            } catch (InterruptedException e){}
            if(ArmMotor.getTargetPosition() == 10) {
                DClaw.setPosition(0.71);
            }
        }
    }

    private void IMUTurnLeft() {
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setPower(0.55);
        back_left.setPower(0.55);
        front_right.setPower(0.55);
        back_right.setPower(0.55);
    }

    private void StopRobot() {
        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }

    private void RotateLeftIMU() {
        Roll = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        while (!(Roll > Angle || isStopRequested())) {
            Roll = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            IMUTurnLeft();
            telemetry.addData("Yaw", Roll);
            telemetry.update();
        }
        StopRobot();
    }

    private void turnLeft180() {
        if (gamepad1.right_bumper) {
            Roll = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            if(Roll < 0){
                while ((Roll < (Roll+180) || isStopRequested())) {
                    Roll = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                    IMUTurnLeft();
                    telemetry.addData("Roll", Roll);
                    telemetry.update();
                }
            }
            StopRobot();
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