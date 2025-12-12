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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

@TeleOp(name="PewPewTeleop", group="TeleOp")
public class PewPewTeleOP extends OpMode {
    double targetRPM = 0;
    double currentRPM, frontLeftPower, frontRightPower, botHeading, backLeftPower, backRightPower, rawHeading, output;
    double mode = 1;
    double kP = 0.0007;
    double kI = 0.0005;
    double kD = 0;
    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    private DcMotor leftFront, leftBack, rightFront, rightBack, noodleIntake, rubberIntakeMotor;
    private DcMotorEx shooterMotor;
    private CRServo gate;
    private IMU imu;
    boolean rubberToggle = false;
    public double headingOffset = 0;
    AprilTagDistanceAndID aprilTagWebcam = new AprilTagDistanceAndID();
    private static final double SHOOTER_KP = 0.0007;
    private static final double SHOOTER_KI = 0.0005;
    private static final double SHOOTER_KD = 0.0;
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

//    double getHeading() {
//        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
//        return angles.getYaw(AngleUnit.DEGREES);
//    }
//
//    public void turnRelative(double deltaAngle) {
//        double startHeading = getHeading();
//        double targetHeading = AngleUnit.normalizeDegrees(startHeading + deltaAngle);
//
//        turnToHeading(targetHeading);
//    }
//
//    public void turnToHeading(double targetAngle) {
//        double Kp = 0.01;
//
//        while (opModeIsActive()) {
//            double current = getHeading();
//            double error = AngleUnit.normalizeDegrees(targetAngle - current);
//
//            if (Math.abs(error) < 2) break;
//
//            double power = Kp * error;
//
//            leftFront.setPower(power);
//            leftBack.setPower(power);
//            rightFront.setPower(-power);
//            rightBack.setPower(-power);
//        }
//
//        leftFront.setPower(0);
//        leftBack.setPower(0);
//        rightFront.setPower(0);
//        rightBack.setPower(0);
//    }

    public void updateShooterPID(double APRIL_TARGET_RPM) {
        currentVelocity = shooterMotor.getVelocity();
        currentRPM = (currentVelocity / 28.0) * 60.0;

        if (APRIL_TARGET_RPM < 200) {
            integralSum = 0;
        }

        now = timer.seconds();
        deltaTime = now - lastTimestamp;
        lastTimestamp = now;

        if (deltaTime <= 0) deltaTime = 0.001;

        error = APRIL_TARGET_RPM - currentRPM;
        integralSum += error * deltaTime;
        derivative = (error - lastError) / deltaTime;

        output = (kP * error) + (kI * integralSum) + (kD * derivative);
        output = Math.max(Math.min(output, 1.0), -1.0);

        shooterMotor.setPower(output);

        lastError = error;
    }

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
    };
//    public void keep_motors_same() {
//        double left_ticks_per_rev = 537.6;
//        double left_wheel_diameter = 31.75; // make sure units match
//        double left_wheel_circum = Math.PI * left_wheel_diameter;
//
//        int left_target_tick = (int)((12 / left_wheel_circum) * left_ticks_per_rev);
//
//        double right_ticks_per_rev = 537.6;
//        double right_wheel_diameter = 31.75; // make sure units match
//        double right_wheel_circum = Math.PI * right_wheel_diameter;
//
//        int right_target_tick = (int)((12 / right_wheel_circum) * right_ticks_per_rev);
//
//
//        leftRubberIntakeMotor.setTargetPosition(left_target_tick);
//        leftRubberIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftRubberIntakeMotor.setPower(0.5);
//
//        rightRubberIntakeMotor.setTargetPosition(right_target_tick); // match target
//        rightRubberIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightRubberIntakeMotor.setPower(0.5);
//
//    }


    @Override
    public void init() {

        lastTimestamp = timer.seconds();
        integralSum = 0;
        lastError = 0;

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

        rubberIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
    public void start(){
        targetRPM = 2700;
        updateShooterPID(targetRPM);
    }

    @Override
    public void loop(){

//            if(gamepad1.left_bumper && !isTurning) {
//                isTurning = true;
//                turnTargetHeading = Math.toRadians(36);
//            }
//
//            if(gamepad1.right_bumper && !isTurning) {
//                isTurning = true;
//                turnTargetHeading = Math.toRadians(0);
//            }

            if(gamepad2.dpadLeftWasReleased()) {
                rubberIntakeMotor.setPower(0.8);
            } else if(gamepad2.dpadRightWasReleased()) {
                rubberIntakeMotor.setPower(0);
            }

            double y = -gamepad1.left_stick_x;
            double x = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading = normalizeRadians(rawHeading - headingOffset);

//            if (isTurning) {
//                double turnError = normalizeRadians(turnTargetHeading - botHeading);
//
//                if (Math.abs(turnError) < Math.toRadians(1)) {
//                    isTurning = false;
//                    rx = 0;
//                } else {
//                    rx = turnError * 0.7;
//                }
//            }

            rx = Range.clip(rx, -1, 1);

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

            if (gamepad2.left_bumper) noodleIntake.setPower(0.75);
            else if (gamepad2.right_bumper) noodleIntake.setPower(0);

            if (gamepad1.b) {
                imu.resetYaw();
                headingOffset = Math.toRadians(90);
            }

            if (gamepad2.dpadUpWasReleased()) {
                autoApril = true;
            } else if (gamepad2.dpadDownWasReleased()) {
                autoApril = false;
            }

            if (!autoApril) {
                if (gamepad2.left_trigger > 0.8) {
                    gate.setPower(1);
                } else if (gamepad2.right_trigger > 0.8) {
                    gate.setPower(-1);
                } else {
                    gate.setPower(0);
                }
                if (gamepad2.y) targetRPM = 2700;
                if (gamepad2.b) targetRPM = 2950;
                if (gamepad2.a) targetRPM = 3500;
                if (gamepad2.x) targetRPM = 4500;
                updateShooterPID(targetRPM);
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

//                    d = det24.ftcPose.y;
//
//                    det24x = (det24.ftcPose.x / 2.54) * -1;
//                    double yaw = det24.ftcPose.yaw;
//
//
//                    telemetry.addData("Yaw off: ", det24.ftcPose.yaw);
//                    telemetry.addData("Angle off: ", angleOff);
//                    double a = Math.toDegrees(Math.atan(det24x/d));
//                    double b = Math.toRadians(36) - Math.toRadians(a);
//                    double angleOff = 90 - Math.toDegrees(b);

                    dInches = d / 2.54;
                    double rpm = aprilTagWebcam.getShooterRPM(dInches);
                    updateShooterPID(rpm);
                } else {
                    gate.setPower(0);
                }

                if (Math.abs(currentRPM - rpm) < 150 && det24 != null) {
                    gate.setPower(-1);
                } else {
                    gate.setPower(1);
                }

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
