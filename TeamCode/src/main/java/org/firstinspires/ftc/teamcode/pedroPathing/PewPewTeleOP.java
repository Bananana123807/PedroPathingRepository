package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import android.util.Size;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name="PewPewTeleop", group="TeleOp")
public class PewPewTeleOP extends LinearOpMode {

    double targetRPM = 0;
    double currentRPM, frontLeftPower, frontRightPower, botHeading, backLeftPower, backRightPower, rawHeading, output;
    private PredominantColorProcessor.Result result;
    double mode = 1;
    double kP = 0.0007;
    double kI = 0.0005;
    double kD = 0;
    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    private DcMotor leftFront, leftBack, rightFront, rightBack, noodleIntake;
    private DcMotorEx shooterMotor;
    private CRServo gate;
    private Servo server;

    private IMU imu;
    public double headingOffset = 0;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        noodleIntake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(CRServo.class, "gate");
        server = hardwareMap.get(Servo.class, "server");

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));

        sleep(500);
        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2;

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE
                )
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        String ballColor = "";
        Timer pathTimer = new Timer();
        pathTimer.resetTimer();

        while (opModeIsActive() || opModeInInit()) {
            double elapsedTime = pathTimer.getElapsedTime();

            result = colorSensor.getAnalysis();
            if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
                ballColor = "Green";
            } else if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
                ballColor = "Purple";
            } else {
                ballColor = "None";
            }

            if ((mode == 1 && (ballColor.equals("Green") || ballColor.equals("Purple"))) ||
                    (mode == 2 && ballColor.equals("Green")) ||
                    (mode == 3 && ballColor.equals("Purple"))) {
                if (elapsedTime >= 500) {
                    server.setPosition(0.85);
                    ballColor = "null";
                    pathTimer.resetTimer();
                }
            } else {
                server.setPosition(0.4);
            }

            double y = -gamepad1.left_stick_x;
            double x = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading = normalizeRadians(rawHeading - headingOffset);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            if (gamepad2.right_trigger == 1) gate.setPower(-1);
            else if (gamepad2.left_trigger == 1) gate.setPower(1);
            else gate.setPower(0);

            if (gamepad2.y) targetRPM = 2700;
            if (gamepad2.b) targetRPM = 2950;
            if (gamepad2.a) targetRPM = 3500;
            if (gamepad2.x) targetRPM = 4500;

            double currentVelocity = shooterMotor.getVelocity();
            currentRPM = (currentVelocity / 28.0) * 60.0;

            double error = targetRPM - currentRPM;
            double deltaTime = timer.seconds();
            integralSum += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;

            output = (kP * error) + (kI * integralSum) + (kD * derivative);
            shooterMotor.setPower(output);

            lastError = error;
            timer.reset();

            if (gamepad2.left_bumper) noodleIntake.setPower(-0.75);
            else if (gamepad2.right_bumper) noodleIntake.setPower(0);

            if (gamepad2.dpad_down || gamepad2.dpad_up) mode = 1;
            if (gamepad2.dpad_right) mode = 2;
            if (gamepad2.dpad_left) mode = 3;

            if (gamepad1.right_bumper) {
                imu.resetYaw();
                headingOffset = Math.toRadians(90);  // Corrected to +90 degrees
            }


            telemetry.addData("Raw Heading (deg)", Math.toDegrees(rawHeading));
            telemetry.addData("Heading Offset (deg)", Math.toDegrees(headingOffset));
            telemetry.addData("Adjusted Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("LF Power", frontLeftPower);
            telemetry.addData("LB Power", backLeftPower);
            telemetry.addData("RF Power", frontRightPower);
            telemetry.addData("RB Power", backRightPower);
            telemetry.addData("Best Match", result.closestSwatch);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Flywheel Power", output);
            telemetry.addData("Ball Color", ballColor);
            telemetry.addData("Mode", mode);
            telemetry.update();
        }
    }
}
