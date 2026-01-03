package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedSideFarAuto", group = "Over-caffeinated")
public class RedSideFarAuto extends OpMode {
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 3550;
    private static final double MAX_INTEGRAL = 500.0;
    private ElapsedTime timer = new ElapsedTime();
    private double currentVelocity, currentRPM, error, lastError, integralSum, derivative, output, deltaTime;
    private double lastTimestamp = 0.0;

    // Hardware
    private DcMotorEx shooterMotor;
    private CRServo gate;
    private DcMotor noodleIntake, rubberIntakeMotor;

    // Pathing
    private Follower follower;
    private Timer pathTimer, ballTimer;
    private int pathState = 0;
    private int counter = 0;
    private boolean isShooting = false;
    private double waitTime = 0;
    private int ballNum = 0;

    // Shooter readiness
    private double rpmStableStart = 0;

    // Poses
    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(80, 90, Math.toRadians(44));
    private final Pose set3frontPose = new Pose(90, 60, Math.toRadians(0));
    private final Pose set3controlPoint = new Pose(80, 41);
    private final Pose set3pickPose = new Pose(165, 63, Math.toRadians(0));
    private final Pose moveOutPose = new Pose(10, 10, Math.toRadians(90));

    // Paths
    private Path scorePreload;
    private PathChain frontSet3, pickupSet3, scoreSet3, moveOut;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45));

        frontSet3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, set3controlPoint, set3frontPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), set3frontPose.getHeading())
                .build();

        pickupSet3 = follower.pathBuilder()
                .addPath(new BezierLine(set3frontPose, set3pickPose))
                .setConstantHeadingInterpolation(set3pickPose.getHeading())
                .build();

        scoreSet3 = follower.pathBuilder()
                .addPath(new BezierLine(set3pickPose, scorePose))
                .setLinearHeadingInterpolation(set3pickPose.getHeading(), scorePose.getHeading())
                .build();

        moveOut = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, moveOutPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), moveOutPose.getHeading())
                .build();
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

        if (elapsed >= 500 && elapsed < 1000) {
            gate.setPower(-1);
        } else if (elapsed >= 1300) {
            gate.setPower(0);
            isShooting = false;
        }
    }

    // Tuned PID + Feedforward constants
    private static final double SHOOTER_KP = 0.0008;   // proportional gain
    private static final double SHOOTER_KI = 0.00015;  // integral gain
    private static final double SHOOTER_KD = 0.0;      // derivative often unnecessary
    private static final double SHOOTER_KF = 0.00027;  // feedforward tuned to motor

    public void updateShooterPID() {
        currentVelocity = shooterMotor.getVelocity();
        currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;

        error = TARGET_RPM - currentRPM;

        double currentTime = timer.seconds();
        deltaTime = currentTime - lastTimestamp;
        lastTimestamp = currentTime;
        if (deltaTime <= 0) return;

        // Integral with anti-windup
        integralSum += error * deltaTime;
        integralSum = Math.max(Math.min(integralSum, MAX_INTEGRAL), -MAX_INTEGRAL);

        // Derivative (optional, kept at 0 here)
        derivative = (error - lastError) / deltaTime;

        // Feedforward + PID
        output = (SHOOTER_KF * TARGET_RPM) +
                (SHOOTER_KP * error) +
                (SHOOTER_KI * integralSum) +
                (SHOOTER_KD * derivative);

        // Clamp to motor power range
        output = Math.max(Math.min(output, 1.0), -1.0);
        shooterMotor.setPower(output);

        lastError = error;
    }

    public boolean isShooterReady() {
        return Math.abs(currentRPM - TARGET_RPM) <= 100;
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                follower.followPath(scorePreload);
                telemetry.addLine("Moving back to score preload");
                waitTime = 1000;
                break;
            case 2: case 3: case 4:
                if (isShooterReady()) {
                    shootBall();
                    telemetry.addLine("Scoring preload balls");
                    waitTime = 100;
                } else {
                    counter -= 1;
                }
                break;
            case 5:
                waitTime = 100;
                break;
            case 6:
                rubberIntakeMotor.setPower(1);
                follower.followPath(frontSet3);
                telemetry.addLine("Driving to the front of set 3");
                waitTime = 1500;
                break;
            case 7:
                follower.setMaxPower(1);
                follower.followPath(pickupSet3);
                telemetry.addLine("Picking up set 3");
                waitTime = 4000;
                break;
            case 8:
                follower.setMaxPower(1);
                follower.followPath(scoreSet3);
                telemetry.addLine("Driving to score set 3");
                waitTime = 2000;
                break;
            case 9: case 10: case 11:
                if (isShooterReady()) {
                    shootBall();
                    waitTime = 100;
                } else {
                    counter -= 1;
                }
                break;
            case 12:
                follower.followPath(moveOut);
                telemetry.addLine("Parking");
                waitTime = 1000;
                break;
            case 13:
                gate.setPower(0);
                shooterMotor.setPower(0);
                noodleIntake.setPower(0);
                rubberIntakeMotor.setPower(0);
                telemetry.addLine("Auto Complete");
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
        noodleIntake.setPower(1);
    }

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gate = hardwareMap.get(CRServo.class, "gate");
        noodleIntake = hardwareMap.get(DcMotor.class, "intake");
        rubberIntakeMotor = hardwareMap.get(DcMotor.class, "x-odo");
        rubberIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        pathTimer = new Timer();
        ballTimer = new Timer();

        pathTimer.resetTimer();
        ballTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        telemetry.update();
    }
}
