package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueSideFarShootAuto", group = "Over-caffeinated")
public class BlueSideFarShootAuto extends OpMode {

    // PID constants
    private static final double SHOOTER_KP = 0.0007;
    private static final double SHOOTER_KI = 0.0005;
    private static final double SHOOTER_KD = 0.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 5200;
    private static final double MAX_INTEGRAL = 500.0; // anti-windup clamp
    private ElapsedTime timer = new ElapsedTime();
    private double currentVelocity, currentRPM, error, lastError, integralSum, derivative, output, deltaTime;
    private double lastTimestamp = 0.0;

    // Servo positions
    private static final double DEFAULT_GATE_POSITION = 0.4;
    private static final double ACTIVE_GATE_POSITION = 0.85;

    // Hardware
    private DcMotorEx shooterMotor;
    private CRServo gate;
    private DcMotor noodleIntake;
    private Servo server;

    // Pathing
    private Follower follower;
    private Timer pathTimer, ballTimer, pickTimer;
    private int pathState = 0;
    private int counter = 0;
    private boolean isShooting = false;
    private double waitTime = 0;
    private int ballNum = 0;

    // Poses
    private final Pose startPose = new Pose(144 - 88, 3, Math.toRadians(180 - 90));
    private final Pose scorePose = new Pose(144 - 83.952, 14.2292, Math.toRadians(120));
    private final Pose outPose = new Pose(144 - 135, 23, Math.toRadians(180));
    private Path scorePath, outPath;

    public void buildPaths() {
        scorePath = new Path(new BezierLine(startPose, scorePose));
        scorePath.setConstantHeadingInterpolation(scorePose.getHeading());

        outPath = new Path(new BezierLine(scorePose, outPose));
        outPath.setConstantHeadingInterpolation(outPose.getHeading());
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void shootBall() {
        if (!isShooting) {
            ballNum++;
            gate.setPower(1);
            ballTimer.resetTimer();
            isShooting = true;
        }

        double elapsed = ballTimer.getElapsedTime();

        if (elapsed >= 900 && elapsed < 1400) {
            gate.setPower(-1);
        } else if (elapsed >= 1300) {
            gate.setPower(0);
            isShooting = false;
        }
    }

    public void updateShooterPID() {
        currentVelocity = shooterMotor.getVelocity();
        currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;

        error = TARGET_RPM - currentRPM;

        double currentTime = timer.seconds();
        deltaTime = currentTime - lastTimestamp;
        lastTimestamp = currentTime;

        if (deltaTime <= 0) return;

        integralSum += error * deltaTime;
        integralSum = Math.max(Math.min(integralSum, MAX_INTEGRAL), -MAX_INTEGRAL);

        derivative = (error - lastError) / deltaTime;

        output = (SHOOTER_KP * error) + (SHOOTER_KI * integralSum) + (SHOOTER_KD * derivative);
        output = Math.max(Math.min(output, 1.0), -1.0);

        shooterMotor.setPower(output);
        lastError = error;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                updateShooterPID();
                follower.followPath(scorePath);
                telemetry.addLine("ScorePath");
                waitTime = 3000;
                break;
            case 2: case 3: case 4:
                shootBall();
                waitTime = 3000;
                break;
            case 5:
                follower.followPath(outPath);
                telemetry.addLine("Finish");
                waitTime = 1500;
                break;
        }
    }

    @Override
    public void loop() {
        updateShooterPID();
        follower.update();

        double elapsedTime = pathTimer.getElapsedTime();

        if (isShooting) {
            shootBall();
        }

        if (elapsedTime >= waitTime && !isShooting) {
            counter += 1;
            setPathState(counter);
            autonomousPathUpdate();
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.addData("Shooter RPM", currentRPM);
        telemetry.addData("Shooting Ball Number", ballNum);
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        noodleIntake.setPower(-0.67);
    }

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gate = hardwareMap.get(CRServo.class, "gate");
        noodleIntake = hardwareMap.get(DcMotor.class, "intake");
        server = hardwareMap.get(Servo.class, "server");

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        pathTimer = new Timer();
        ballTimer = new Timer();
        pickTimer = new Timer();

        pathTimer.resetTimer();
        ballTimer.resetTimer();
        pickTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        telemetry.update();
    }
}
