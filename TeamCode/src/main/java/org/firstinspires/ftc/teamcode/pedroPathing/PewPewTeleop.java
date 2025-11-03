package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="PewPewPewPewTeleop", group="TeleOp")
public class PewPewTeleop extends OpMode {

    // Motors and servos
    private DcMotor leftFront, leftBack, rightFront, rightBack, noodleIntake;
    private DcMotorEx shooterMotor;
    private CRServo gate, feeder;
    private Servo server;

    // IMU and heading offset
    private IMU imu;
    private int toggle = 0;
    public double headingOffset = 0;

    @Override
    public void init() {
        // Hardware mapping
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        noodleIntake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(CRServo.class, "gate");
        feeder = hardwareMap.get(CRServo.class, "intakeServo");
        server = hardwareMap.get(Servo.class, "server");

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
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));

        // Wait briefly to allow IMU to stabilize
        sleep(500);

        // Read initial heading and store offset
        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        headingOffset = headingOffset + (Math.PI)/2;
    }

    @Override
    public void loop() {
        // Gamepad inputs
        double y = -gamepad1.left_stick_y;  // Forward positive
        double x = gamepad1.left_stick_x;   // Strafe right positive
        double rx = gamepad1.right_stick_x; // Rotation

        // Read raw IMU heading and adjust with offset
        double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = rawHeading-headingOffset;
        botHeading = normalizeRadians(botHeading);

        // Field-centric transform
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Motor power calculations
        double denominator = -1*(Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0));
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
        if (gamepad2.dpad_left) {
            gate.setPower(-1);
        } else if (gamepad2.dpad_right) {
            gate.setPower(1);
        } else {
            gate.setPower(0);
        }
        //Noodle Intake control
        if (gamepad2.right_trigger == 1){
            noodleIntake.setPower(-0.67);
        } else if (gamepad2.left_trigger == 1){
            noodleIntake.setPower(1);
        }
        //Servo Intake and Server control
        if (gamepad2.dpad_up){
            feeder.setPower(-1);
        } else if (gamepad2.dpad_down){
            feeder.setPower(1);
        }

        if (gamepad2.right_bumper){
            server.setPosition(0.85);
        } else if (gamepad2.left_bumper){
            server.setPosition(0.4);
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

        if (gamepad1.right_bumper) {
            imu.resetYaw();
            headingOffset = 0;
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
