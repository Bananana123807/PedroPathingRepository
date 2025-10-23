package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="PewPewTeleop", group="TeleOp")
public class PewPewTeleop extends OpMode {

    // Motors and servos
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx shooterMotor;
    private CRServo gate;

    // IMU and heading offset
    private IMU imu;
    private double headingOffset = 0;

    @Override
    public void init() {
        // Hardware mapping
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        gate = hardwareMap.get(CRServo.class, "gate");

        // Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        // Brake behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU initialization with robot orientation
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        // Wait briefly to allow IMU to stabilize
        sleep(500);

        // Read initial heading and store offset
        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void loop() {
        // Gamepad inputs
        double y = -gamepad1.left_stick_y;  // Forward positive
        double x = gamepad1.left_stick_x;   // Strafe right positive
        double rx = gamepad1.right_stick_x; // Rotation

        // Read raw IMU heading and adjust with offset
        double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = rawHeading - headingOffset;
        botHeading = normalizeRadians(botHeading);

        // Field-centric transform
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Motor power calculations
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        // Gate control
        if (gamepad1.dpad_left) {
            gate.setPower(-1);
        } else if (gamepad1.dpad_right) {
            gate.setPower(1);
        } else {
            gate.setPower(0);
        }

        // Shooter control (simplified here; add your PID logic if you want)
        // Example: set power directly with buttons
        if (gamepad2.y) {
            shooterMotor.setPower(0.7);
        } else if (gamepad2.b) {
            shooterMotor.setPower(0.85);
        } else if (gamepad2.a) {
            shooterMotor.setPower(1.0);
        } else {
            shooterMotor.setPower(0);
        }

        // Telemetry
        telemetry.addData("Raw Heading (deg)", Math.toDegrees(rawHeading));
        telemetry.addData("Heading Offset (deg)", Math.toDegrees(headingOffset));
        telemetry.addData("Adjusted Heading (deg)", Math.toDegrees(botHeading));
        telemetry.addData("LF Power", frontLeftPower);
        telemetry.addData("LB Power", backLeftPower);
        telemetry.addData("RF Power", frontRightPower);
        telemetry.addData("RB Power", backRightPower);
        telemetry.update();
    }

    // Utility to keep angle between -pi and pi
    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    // Sleep helper
    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
