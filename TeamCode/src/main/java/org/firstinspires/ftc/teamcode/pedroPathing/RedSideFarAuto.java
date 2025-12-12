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

    // PID constants
    private static final double SHOOTER_KP = 0.0007;
    private static final double SHOOTER_KI = 0.0005;
    private static final double SHOOTER_KD = 0.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 3700;
    private static final double MAX_INTEGRAL = 500.0; // anti-windup clamp
    private int TARGET_RPM2 = 3700;
    private ElapsedTime timer = new ElapsedTime();
    private double currentVelocity, currentRPM, error, lastError, integralSum, derivative, output, deltaTime;
    private double lastTimestamp = 0.0;

    // Hardware
    private DcMotorEx shooterMotor;
    private CRServo gate;
    private DcMotor noodleIntake, rubberIntakeMotor;
    double now;

    // Pathing
    private Follower follower;
    private Timer pathTimer, ballTimer;
    private int pathState = 0;
    private int counter = 0;
    private boolean isShooting = false;
    private double currentTargetRPM = TARGET_RPM;
    private double waitTime = 0;
    private int ballNum = 0;

    // PID variables
    //private double currentVelocity, currentRPM, error, lastError, integralSum, derivative, output, deltaTime;

    // Poses
    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(80, 90, Math.toRadians(45));
    private final Pose set3frontPose = new Pose(90, 55, Math.toRadians(0));
    private final Pose set3controlPoint = new Pose(80, 41);
    private final Pose set3pickPose = new Pose(160, 60, Math.toRadians(0));
    private final Pose moveOutPose = new Pose(10, 10, Math.toRadians(90));

    // Paths
    private Path scorePreload;
    private PathChain frontSet3, pickupSet3, scoreSet3, frontSet2, shake, shakeBack, pickupSet2, scoreSet2, moveOut;

    private Pose computeRetreatPose(Pose origin, double distanceInches) {
        double heading = origin.getHeading();
        double dx = -distanceInches * Math.cos(heading);
        double dy = -distanceInches * Math.sin(heading);
        return new Pose(origin.getX() + dx, origin.getY() + dy, heading);
    }

    public void buildPaths() {
        //Score Preload (backwards 30 inches)
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45));

        //Drive in front of set 3
        frontSet3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, set3controlPoint, set3frontPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), set3frontPose.getHeading())
                .build();

        //Pickup set 3
        pickupSet3 = follower.pathBuilder()
                .addPath(new BezierLine(set3frontPose, set3pickPose))
                .setConstantHeadingInterpolation(set3pickPose.getHeading())
                .build();

        //Score set 3
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

        if (elapsed >= 1000 && elapsed < 1500) {
            gate.setPower(-1);
        } else if (elapsed >= 1500) {
            gate.setPower(0);
            isShooting = false;
        }
    }

    public void updateShooterPID(double rpm) {
        currentVelocity = shooterMotor.getVelocity();
        currentRPM = (currentVelocity / 28.0) * 60.0;

        if (rpm < 200) {
            integralSum = 0;
        }

        now = timer.seconds();
        deltaTime = now - lastTimestamp;
        lastTimestamp = now;

        if (deltaTime <= 0) deltaTime = 0.001;

        error = rpm - currentRPM;
        integralSum += error * deltaTime;
        derivative = (error - lastError) / deltaTime;

        output = (SHOOTER_KP * error) + (SHOOTER_KI * integralSum) + (SHOOTER_KD * derivative);
        output = Math.max(Math.min(output, 1.0), -1.0);

        shooterMotor.setPower(output);

        lastError = error;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                updateShooterPID(TARGET_RPM);
                follower.followPath(scorePreload);
                telemetry.addLine("Moving back to score preload");
                waitTime = 2000;
                break;
            case 2: case 3: case 4:
                if (currentRPM < 150 + TARGET_RPM && TARGET_RPM - 150 < currentRPM) {
                    shootBall();
                    telemetry.addLine("Scoring preload balls");
                    waitTime = 300;
                } else {
                    if (TARGET_RPM - 150 > currentRPM) {
                        TARGET_RPM2 += 100;
                        currentTargetRPM = TARGET_RPM2;
                        waitTime = 500;
                    } else if (TARGET_RPM + 150 < currentRPM) {
                        TARGET_RPM2 -= 100;
                        currentTargetRPM = TARGET_RPM2;
                        waitTime = 500;
                    }
                    counter -= 1;
                }
                break;
            case 5:
                waitTime = 1000;
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
                shootBall();
                waitTime = 100;
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

        updateShooterPID(currentTargetRPM);
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
