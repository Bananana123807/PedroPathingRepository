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
import org.opencv.core.Mat;

@Autonomous(name = "BlueSideCloseAuto", group = "Over-caffeinated")
public class BlueSideCloseAuto extends OpMode {

    private Path turnToClassifier;

    private double shooterPower = -0.55;
    private double gatePower = -1;
    private Follower follower;
    private Timer pathTimer;
    private DcMotorEx shooterMotor = null;
    private CRServo gate = null;
    private IMU imu = null;

    private int pathState;
    private final int TICKS_PER_REV = 28;

    private double TARGET_RPM = 2900;
    private double currentRPM = 0;

    private double kP = 0.005;
    private double kI = 0.00008;
    private double kD = 0;

    private double previousError = 0;
    private double integral = 0;
    private int counter = -1;
    private double output = 0;

    private Pose startPose;
    private Pose backwardPose;
    private Pose exitPose;

    private Path moveBackward;
    private PathChain moveOut;

    private double waitTime = 2000;

    public void buildPaths() {
        moveBackward = new Path(new BezierLine(startPose, backwardPose));
        moveBackward.setLinearHeadingInterpolation(startPose.getHeading(), backwardPose.getHeading());

        moveOut = follower.pathBuilder()
                .addPath(new BezierLine(backwardPose, exitPose))
                .setLinearHeadingInterpolation(backwardPose.getHeading(), Math.toRadians(45))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(moveBackward);
                telemetry.addLine("Backing up");
                waitTime = 2000;
                break;
            case 1:
                gate.setPower(1);
                waitTime = 1500;
                break;
            case 2:
                shooterMotor.setPower(output);
                gate.setPower(gatePower);
                waitTime = 500;
                break;
            case 3:
                gate.setPower(1);
                waitTime = 1500;
                break;
            case 4:
                gate.setPower(gatePower);
                waitTime = 500;
                break;
            case 5:
                gate.setPower(1);
                waitTime = 1500;
                break;
            case 6:
                gate.setPower(gatePower);
                waitTime = 500;
                break;
            case 7:
                gate.setPower(0);
                TARGET_RPM = 00;
                waitTime = 500;
                break;
            case 8:
                follower.followPath(moveOut);
                telemetry.addLine("Exiting Zone");
                waitTime = 2000;
                break;
        }
    }

    @Override
    public void loop() {
        if (pathState == 9 && turnToClassifier != null) {
            follower.followPath(turnToClassifier);
        }

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
        imuHeading -= Math.PI / 2; // 90° counterclockwise turn for blue side
        telemetry.addData("IMU Initial Heading (rad)", imuHeading);

        startPose = new Pose(0, 0, imuHeading);

        double backX = -20 * Math.cos(imuHeading);
        double backY = -20 * Math.sin(imuHeading);
        backwardPose = new Pose(backX, backY, imuHeading);

        double strafeDistance = 15;
        double strafeX = backwardPose.getX() - strafeDistance * Math.cos(imuHeading - Math.PI / 2);
        double strafeY = backwardPose.getY() - strafeDistance * Math.sin(imuHeading - Math.PI / 2);
        exitPose = new Pose(strafeX, strafeY + 5, imuHeading);

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
