package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.util.Range;
@TeleOp(name="REDPewPewTeleOP", group="TeleOp")
public class REDPewPewTeleOP extends OpMode {
    double targetRPM = 0;
    double currentRPM, frontLeftPower, frontRightPower, botHeading, backLeftPower, backRightPower, rawHeading, output;
    double velocityY = 1400;
    double velocityB = 1500;
    double velocityA = 1700;
    double velocityX = 600;
    double curTargetVelocity = velocityY;
    double F = 12.504;
    double P = 45.62;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 0;
    ElapsedTime timer = new ElapsedTime();
    private DcMotor leftFront, leftBack, rightFront, rightBack, noodleIntake, rubberIntakeMotor;
    private DcMotorEx shooterMotor;
    private CRServo gate;
    private IMU imu;
    boolean rubberToggle = false;
    public double headingOffset = 0;
    AprilTagDistanceAndID aprilTagWebcam = new AprilTagDistanceAndID();
    private static final double TICKS_PER_REV = 28.0;
    private ElapsedTime apriltimer = new ElapsedTime();
    private double currentVelocity, now, error, derivative, deltaTime;
    private double lastTimestamp = 0.0;
    double d = 0;
    private String ball1 = "null";
    private String ball2 = "null";
    private String ball3 = "null";
    private int id;
    private boolean isTurning = false;
    private double turnTargetHeading;
    double det24x;
    double det24yaw;
    boolean autoApril = false;
    double dInches = 0;
    double rpm = 0;
    double rx;
    public void displayBallColorTelemetry(int detectedID){
        if (detectedID == 21) {
            ball1 = "Green";
            ball2 = "Purple";
            ball3 = "Purple";
        } else if (detectedID == 22) {
            ball1 = "Purple";
            ball2 = "Green";
            ball3 = "Purple";
        } else if (detectedID == 23) {
            ball1 = "Purple";
            ball2 = "Purple";
            ball3 = "Green";
        }

        telemetry.addData("ball1Color: ", ball1);
        telemetry.addData("ball2Color: ", ball2);
        telemetry.addData("ball3Color: ", ball3);
    }

    @Override
    public void init() {

        lastTimestamp = timer.seconds();

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");

        gate = hardwareMap.get(CRServo.class, "gate");
        aprilTagWebcam.init(hardwareMap, telemetry);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");
        rubberIntakeMotor = hardwareMap.get(DcMotor.class, "x-odo");
        noodleIntake = hardwareMap.get(DcMotor.class, "intake");

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        gate.setDirection(CRServo.Direction.FORWARD);

        rubberIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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

        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2;

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        Timer pathTimer = new Timer();
        pathTimer.resetTimer();

        timer.reset();
    }

    @Override
    public void start(){targetRPM = 900;}

    @Override
    public void loop(){

            if(gamepad2.dpadLeftWasReleased()) {
                rubberIntakeMotor.setPower(-0.85);
            } else if(gamepad2.dpadRightWasReleased()) {
                rubberIntakeMotor.setPower(0.1);
            }

            if (gamepad2.left_bumper) noodleIntake.setPower(1);
            else if (gamepad2.right_bumper) noodleIntake.setPower(0);

            if (gamepad1.b) {
                imu.resetYaw();
                headingOffset = Math.toRadians(90);
            }

            if (gamepad1.dpadUpWasReleased()) {
                autoApril = true;
            } else if (gamepad1.dpadDownWasReleased()) {
                autoApril = false;
            }

            if (!autoApril) {
                rx = gamepad1.right_stick_x;

                if (gamepad2.left_trigger > 0.8) {
                    gate.setPower(1);
                } else if (gamepad2.right_trigger > 0.8) {
                    gate.setPower(-1);
                } else {
                    gate.setPower(0);
                }

                if (gamepad2.yWasPressed()) curTargetVelocity = velocityY;
                if (gamepad2.bWasPressed()) curTargetVelocity = velocityB;
                if (gamepad2.aWasReleased()) curTargetVelocity = velocityA;
                if (gamepad2.xWasReleased()) curTargetVelocity = velocityX;

                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

                shooterMotor.setVelocity(curTargetVelocity);

                double curVelocity = shooterMotor.getVelocity();
                double error = curTargetVelocity - curVelocity;

                telemetry.addData("Target velocity", curTargetVelocity);
                telemetry.addData("Current velocity", "%.2f", curVelocity);

                double y = -gamepad1.left_stick_x;
                double x = -gamepad1.left_stick_y;

                rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                botHeading = normalizeRadians(rawHeading - headingOffset);

                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
                frontLeftPower = (rotY + rotX + rx) / denominator;
                backLeftPower = (rotY - rotX + rx) / denominator;
                frontRightPower = (rotY - rotX - rx) / denominator;
                backRightPower = (rotY + rotX - rx) / denominator;

                if (gamepad1.right_trigger > 0.8){
                    leftFront.setPower(frontLeftPower/2);
                    leftBack.setPower(backLeftPower/2);
                    rightFront.setPower(frontRightPower/2);
                    rightBack.setPower(backRightPower/2);
                } else {
                    leftFront.setPower(frontLeftPower);
                    leftBack.setPower(backLeftPower);
                    rightFront.setPower(frontRightPower);
                    rightBack.setPower(backRightPower);
                }

                rx = Range.clip(rx, -1, 1);
            }

            //APRIL TAG PID DISTANCE

            if (autoApril) {
                aprilTagWebcam.update();

                AprilTagDetection det24 = aprilTagWebcam.getTagBySpecificID(24);
                aprilTagWebcam.displayDetectionTelemetry(det24);

                AprilTagDetection det21 = aprilTagWebcam.getTagBySpecificID(21);

                AprilTagDetection det22 = aprilTagWebcam.getTagBySpecificID(22);

                AprilTagDetection det23 = aprilTagWebcam.getTagBySpecificID(23);

                if (det21 != null) {
                    id = 21;
                }

                if (det22 != null) {
                    id = 22;
                }

                if (det23 != null) {
                    id = 23;
                }

                displayBallColorTelemetry(id);

                if (det24 != null) {

                    if (gamepad2.yWasPressed()) curTargetVelocity = velocityY;
                    if (gamepad2.bWasPressed()) curTargetVelocity = velocityB;
                    if (gamepad2.aWasReleased()) curTargetVelocity = velocityA;
                    if (gamepad2.xWasReleased()) curTargetVelocity = velocityX;

                    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                    shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

                    shooterMotor.setVelocity(curTargetVelocity);

//                    d = det24.ftcPose.y;

//                    dInches = d / 2.54;
//                    double rpm = aprilTagWebcam.getShooterRPM(dInches);
//
//                    shooterMotor.setVelocity(rpm);
//                    currentRPM = shooterMotor.getVelocity();
//
//                    telemetry.addData("TargetRPM: ", rpm);
//                    telemetry.addData("CurrentRPM: ", currentRPM);
//
//                    if (Math.abs(currentRPM - rpm) < 100) {
//                        gate.setPower(-1);
//                    } else {
//                        gate.setPower(1);
//                    }

                    double yawError = -1*(det24.ftcPose.yaw) - Math.toRadians(8); // radians
                    double kPYaw = 0.02; // tune this

                    // Override joystick rotation with PID correction
                    rx = Range.clip(kPYaw * yawError, -0.5, 0.5);
                    if (Math.abs(yawError) < Math.toRadians(2)) {
                        rx = 0;
                    }

                    // --- Drive math with AprilTag correction ---
                    double y = -gamepad1.left_stick_x;
                    double x = -gamepad1.left_stick_y;

                    rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    botHeading = normalizeRadians(rawHeading - headingOffset);

                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
                    frontLeftPower = (rotY + rotX + rx) / denominator;
                    backLeftPower = (rotY - rotX + rx) / denominator;
                    frontRightPower = (rotY - rotX - rx) / denominator;
                    backRightPower = (rotY + rotX - rx) / denominator;

                    if (gamepad1.right_trigger > 0.8) {
                        leftFront.setPower(frontLeftPower / 2);
                        leftBack.setPower(backLeftPower / 2);
                        rightFront.setPower(frontRightPower / 2);
                        rightBack.setPower(backRightPower / 2);
                    } else {
                        leftFront.setPower(frontLeftPower);
                        leftBack.setPower(backLeftPower);
                        rightFront.setPower(frontRightPower);
                        rightBack.setPower(backRightPower);
                    }

                    telemetry.addData("Yaw off: ", yawError);

                } else {
                    rx = gamepad1.right_stick_x;

                    gate.setPower(0);

                    double y = -gamepad1.left_stick_x;
                    double x = -gamepad1.left_stick_y;

                    rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    botHeading = normalizeRadians(rawHeading - headingOffset);

                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
                    frontLeftPower = (rotY + rotX + rx) / denominator;
                    backLeftPower = (rotY - rotX + rx) / denominator;
                    frontRightPower = (rotY - rotX - rx) / denominator;
                    backRightPower = (rotY + rotX - rx) / denominator;

                    if (gamepad1.right_trigger > 0.8){
                        leftFront.setPower(frontLeftPower/2);
                        leftBack.setPower(backLeftPower/2);
                        rightFront.setPower(frontRightPower/2);
                        rightBack.setPower(backRightPower/2);
                    } else {
                        leftFront.setPower(frontLeftPower);
                        leftBack.setPower(backLeftPower);
                        rightFront.setPower(frontRightPower);
                        rightBack.setPower(backRightPower);
                    }
                }

                if (gamepad2.yWasPressed()) curTargetVelocity = velocityY;
                if (gamepad2.bWasPressed()) curTargetVelocity = velocityB;
                if (gamepad2.aWasReleased()) curTargetVelocity = velocityA;
                if (gamepad2.xWasReleased()) curTargetVelocity = velocityX;

                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

                shooterMotor.setVelocity(curTargetVelocity);

                telemetry.addData("Distance (in)", dInches);
                telemetry.addData("Output", output);
                telemetry.addData("Target RPM", rpm);
                telemetry.addData("Current RPM", currentRPM);
            }

            telemetry.addData("Raw Heading (deg)", Math.toDegrees(rawHeading));
            telemetry.addData("Heading Offset (deg)", Math.toDegrees(headingOffset));
            telemetry.addData("Adjusted Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("LF Power", frontLeftPower);
            telemetry.addData("LB Power", backLeftPower);
            telemetry.addData("RF Power", frontRightPower);
            telemetry.addData("RB Power", backRightPower);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Flywheel Power", output);
            telemetry.update();
    }
}
