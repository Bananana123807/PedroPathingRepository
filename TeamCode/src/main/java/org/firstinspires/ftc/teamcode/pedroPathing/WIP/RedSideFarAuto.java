package org.firstinspires.ftc.teamcode.pedroPathing.WIP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@Autonomous(name = "RedSideFarAuto2", group = "Over-caffeinated")
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

    private double TARGET_RPM = 3000;
    private double currentRPM = 0;

    private double kP = 0.005;
    private double kI = 0.00008;
    private double kD = 0;

    private double previousError = 0;
    private double integral = 0;
    private int counter = -1;
    private double output = 0;

    private Pose startPose;
    private Pose forwardPose;
    private final Pose turnPose = new Pose(20, 0, Math.toRadians(-26));
    private final Pose exitPose = new Pose(40, 0);

    private Path moveForward;
    private Path turnToShoot;
    private PathChain moveOut;

    private double waitTime = 2000;

    public void buildPaths() {
        moveForward = new Path(new BezierLine(startPose, forwardPose));
        moveForward.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());

        turnToShoot = new Path(new BezierLine(forwardPose, turnPose));
        turnToShoot.setLinearHeadingInterpolation(forwardPose.getHeading(), turnPose.getHeading());

        moveOut = follower.pathBuilder()
                .addPath(new BezierLine(turnPose, exitPose))
                .setConstantHeadingInterpolation(turnPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move forward
                follower.followPath(moveForward);
                telemetry.addLine("Moving Forward");
                waitTime = 2000;
                break;
            case 1: // Turn to shoot
                follower.followPath(turnToShoot);
                telemetry.addLine("Turning to Shoot");
                waitTime = 1000;
                break;
            case 2: // Shoot ball 1

                shooterMotor.setPower(output);
                gate.setPower(gatePower);
                waitTime = 1500;
                break;
            case 3: // Shoot ball 2
                gate.setPower(1);
                waitTime = 2500;
                break;
            case 4: // Shoot ball 3
                gate.setPower(gatePower);
                waitTime = 1500;
                break;
            case 5: // Stop shooter
                gate.setPower(0);
                shooterMotor.setPower(0);
                waitTime = 500;
                break;
            case 6: // Move out
                follower.followPath(moveOut);
                telemetry.addLine("Exiting Zone");
                waitTime = 2000;
                break;
        }
    }

    @Override
    public void loop() {
        double targetTicksPerSec = (TARGET_RPM / 60.0) * TICKS_PER_REV;
        double currentTicksPerSec = shooterMotor.getVelocity();
        currentRPM = (currentTicksPerSec / TICKS_PER_REV) * 60.0;

        double error = targetTicksPerSec - currentTicksPerSec;
        integral += error * 0.05;
        double derivative = (error - previousError) / 0.05;

        output = (kP * error) + (kI * integral) + (kD * derivative);
        output = Math.max(0, Math.min(1, output));
        previousError = error;

        follower.update();

        double elapsedTime = pathTimer.getElapsedTime();
        shooterMotor.setPower(output);

        if (elapsedTime >= waitTime) {
            counter += 1;
            setPathState(counter);
            autonomousPathUpdate();
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

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("IMU Initial Heading (rad)", imuHeading);

        startPose = new Pose(0, 0, imuHeading);

        // Move forward 20 units along heading
        double dx = 20 * Math.cos(imuHeading);
        double dy = 20 * Math.sin(imuHeading);
        forwardPose = new Pose(dx, dy, imuHeading);

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
