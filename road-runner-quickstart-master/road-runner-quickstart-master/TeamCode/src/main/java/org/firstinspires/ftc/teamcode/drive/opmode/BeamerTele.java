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
    private DcMotor StringMotor;

    int Fast;

    float Yaw;

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

    /**
     * Describe this function...
     */
    private void Telemetry() {
        telemetry.addData("BackRightPos", back_right.getCurrentPosition());
        telemetry.addData("BackLeftPos", back_left.getCurrentPosition());
        telemetry.addData("Arm Target", ArmMotor.getTargetPosition());
        telemetry.addData("Arm Current", ArmMotor.getCurrentPosition());
        telemetry.addData("Yaw", IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).thirdAngle);
        telemetry.update();
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        StringMotor = hardwareMap.get(DcMotor.class, "StringMotor");
        InitIMU();
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Fast = 1;
        waitForStart();
        ArmMotor.setTargetPosition(250);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) ArmMotor).setVelocity(-750);
        while (opModeIsActive() && ArmMotor.isBusy()) {
            idle();
        }
        while (opModeIsActive()) {
            Telemetry();
            Movement();
            turnLeft180();
            Arm();
            bigArm();
        }
    }

    /**
     * Describe this function...
     */
    private void Arm() {
        if (gamepad2.dpad_up) {
            ArmMotor.setTargetPosition(675);
        } else if (gamepad2.dpad_right) {
            ArmMotor.setTargetPosition(580);
        } else if (gamepad2.dpad_left) {
            ArmMotor.setTargetPosition(370);
        } else if (gamepad2.dpad_down) {
            ArmMotor.setTargetPosition(150);
        }
        if (ArmMotor.getCurrentPosition() < ArmMotor.getTargetPosition() - 16) {
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) ArmMotor).setVelocity(-750);
        } else if (ArmMotor.getCurrentPosition() > ArmMotor.getTargetPosition() + 16) {
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) ArmMotor).setVelocity(400);
        } else if (ArmMotor.getTargetPosition() == 250) {
            ((DcMotorEx) ArmMotor).setVelocity(0);
            ArmMotor.setPower(0);
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
        Yaw = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).thirdAngle;
        while (!(Yaw > Angle || isStopRequested())) {
            Yaw = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).thirdAngle;
            IMUTurnLeft();
            telemetry.addData("Yaw", Yaw);
            telemetry.update();
        }
        StopRobot();
    }

    private void turnLeft180() {
        Angle = 180;
        if (gamepad1.right_bumper) {
            Yaw = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).thirdAngle;
            while ((Yaw > Angle || isStopRequested())) {
                Yaw = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).thirdAngle;
                IMUTurnLeft();
                telemetry.addData("Yaw", Yaw);
                telemetry.update();
            }
        }
    }

    private void bigArm() {
        if (gamepad1.right_trigger>0) {
            ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            ArmMotor.setPower(0.5);
            } else if (gamepad1.left_trigger>0) {
            ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            ArmMotor.setPower(0.5);

        }else{
            ArmMotor.setPower(0);
        }
        ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition());
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