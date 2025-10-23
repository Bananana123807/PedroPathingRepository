package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;

@Autonomous(name = "RedSideFarAuto", group = "Over-caffeinated")
public class RedSideFarAuto extends OpMode {

    private double shooterPower = -0.55;
    private double gatePower = -1;
    private Follower follower;
    private Timer pathTimer;
    private DcMotorEx shooterMotor = null;
    private CRServo gate = null;
    private IMU imu = null;

    private int pathState;
    private final int TICKS_PER_REV = 28;

    private double TARGET_RPM = 0;
    private double currentRPM = 0;

    // PID coefficients for flywheel control
    private double kP = 0.005;  // Proportional gain
    private double kI = 0.00008; // Integral gain
    private double kD = 0;  // Derivative gain

    private double previousError = 0; // Previous error for derivative
    private double integral = 0;       // Integral of error

    private int counter = -1;
    private double output = 0;

    private double imuHeadingOffset = 0;

    private Pose startPose;  // will set in init after reading IMU
    private final Pose scorePose = new Pose(15, 0);
    private final Pose moveOutPose = new Pose(40, 0);

    private Path scorePreload;
    private PathChain moveOut;

    private double waitTime = 2000;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-26));

        moveOut = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, moveOutPose))
                .setConstantHeadingInterpolation(Math.toRadians(-26))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooterMotor.setPower(output);
                follower.followPath(scorePreload);
                telemetry.addLine("Moving Back");
                waitTime = 2000;
                break;
            case 1:
                if (TARGET_RPM - 100 < currentRPM && currentRPM < TARGET_RPM + 100) {
                    shooterMotor.setPower(output);
                    gate.setPower(gatePower);
                    waitTime = 1500;
                } else {
                    counter -= 1;
                }
                break;
            case 2:
                gate.setPower(1);
                waitTime = 2500;
                break;
            case 3:
                if (TARGET_RPM - 100 < currentRPM && currentRPM < TARGET_RPM + 100) {
                    shooterMotor.setPower(output);
                    gate.setPower(gatePower);
                    waitTime = 500;
                } else {
                    counter -= 1;
                }
                break;
            case 4:
                gate.setPower(1);
                waitTime = 2500;
                break;
            case 5:
                if (TARGET_RPM - 100 < currentRPM && currentRPM < TARGET_RPM + 100) {
                    shooterMotor.setPower(output);
                    gate.setPower(gatePower);
                    waitTime = 1500;
                } else {
                    counter -= 1;
                }
                break;
            case 6:
                gate.setPower(0);
                shooterMotor.setPower(0);
                break;
            case 7:
                follower.followPath(moveOut);
                telemetry.addLine("Moving Out");
                break;
        }
    }

    @Override
    public void loop() {
        // Update flywheel control PID
        double targetTicksPerSec = (TARGET_RPM / 60.0) * TICKS_PER_REV;
        double currentTicksPerSec = shooterMotor.getVelocity();
        currentRPM = (currentTicksPerSec / TICKS_PER_REV) * 60.0;

        double error = targetTicksPerSec - currentTicksPerSec;
        integral += error * 0.05;  // Assume loop runs approx every 50ms
        double derivative = (error - previousError) / 0.05;

        output = (kP * error) + (kI * integral) + (kD * derivative);
        output = Math.max(0, Math.min(1, output));  // Clamp between 0 and 1

        previousError = error;

        follower.update();

        double elapsedTime = pathTimer.getElapsedTime();
        shooterMotor.setPower(output);

        if (elapsedTime >= waitTime) {
            counter += 1;
            setPathState(counter);
            autonomousPathUpdate();
            shooterMotor.setPower(output);
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (rad)", follower.getPose().getHeading());
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.addData("Shooter RPM", "%.1f", currentRPM);
        telemetry.addData("Target RPM", "%.1f", TARGET_RPM);
        telemetry.addData("Shooter TPS", "%.1f", currentTicksPerSec);
        telemetry.addData("Output (PID)", "%.3f", output);
        telemetry.update();
    }

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        gate = hardwareMap.get(CRServo.class, "gate");
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize IMU with orientation params
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        // Wait for IMU to stabilize
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Capture IMU initial heading offset (yaw)
        imuHeadingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("IMU Initial Heading (rad)", imuHeadingOffset);

        // Set startPose with heading adjusted to zero using IMU offset
        startPose = new Pose(0, 0, -imuHeadingOffset);

        pathTimer = new Timer();
        pathTimer.resetTimer();

        setPathState(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.update();
    }
}
