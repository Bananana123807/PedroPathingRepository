package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import android.util.Size;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name="PewPewTeleopAutomatic", group="TeleOp")
public class PewPewTeleOpAutomatic extends LinearOpMode {

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
    public void runOpMode() {
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
        rightBack.setDirection(DcMotor.Direction.FORWARD);
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

        /*
         * Build a vision portal to run the Color Sensor process.
         *
         *  - Add the colorSensor process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .build();

        telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        String ballColor = "";
        Timer pathTimer;
        pathTimer = new Timer();
        pathTimer.resetTimer();

        while (opModeIsActive() || opModeInInit()){
            double elapsedTime = pathTimer.getElapsedTime();
            telemetry.addLine("Preview on/off: 3 dots, Camera Stream\n");

        // Request the most recent color analysis.  This will return the closest matching
        // colorSwatch and the predominant color in the RGB, HSV and YCrCb color spaces.
        // The color space values are returned as three-element int[] arrays as follows:
        //  RGB   Red 0-255, Green 0-255, Blue 0-255
        //  HSV   Hue 0-180, Saturation 0-255, Value 0-255
        //  YCrCb Luminance(Y) 0-255, Cr 0-255 (center 128), Cb 0-255 (center 128)
        //
        // Note: to take actions based on the detected color, simply use the colorSwatch or
        // color space value in a comparison or switch.   eg:

        //  or:
        //    if (result.RGB[0] > 128) {... some code  ...}
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();
            if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
                ballColor = "Green";
                telemetry.addData("Color Detected:", ballColor);
            }

            if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
                ballColor = "Purple";
                telemetry.addData("Color Detected:", ballColor);
            }

            if (ballColor.equals("Green") || ballColor.equals("Purple")){
                if(elapsedTime >= 500) {
                    server.setPosition(0.85);
                    ballColor = "null";
                    pathTimer.resetTimer();
                }
            } else {
                server.setPosition(0.4);
            }
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
            if (gamepad2.left_trigger == 1){
                noodleIntake.setPower(0);
            } else {
                noodleIntake.setPower(-0.75);
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
            telemetry.addData("Best Match", result.closestSwatch);
            telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)",
                    result.RGB[0], result.RGB[1], result.RGB[2]));
            telemetry.addLine(String.format("HSV   (%3d, %3d, %3d)",
                    result.HSV[0], result.HSV[1], result.HSV[2]));
            telemetry.addLine(String.format("YCrCb (%3d, %3d, %3d)",
                    result.YCrCb[0], result.YCrCb[1], result.YCrCb[2]));
            telemetry.update();
    }

    // Utility to keep angle between -pi and pi
//    private double normalizeRadians(double angle) {
//        while (angle > Math.PI) angle -= 2.0 * Math.PI;
//        while (angle < -Math.PI) angle += 2.0 * Math.PI;
//        return angle;
//    }

    // Sleep helper
//    private void sleep(long millis) {
//        try {
//            Thread.sleep(millis);
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt();
//        }
    }
}
