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

@Autonomous(name = "RedSideCloseAuto", group = "Over-caffeinated")
public class RedSideCloseAuto extends OpMode {

    // PID constants
    private static final double SHOOTER_KP = 0.0007;
    private static final double SHOOTER_KI = 0.0005;
    private static final double SHOOTER_KD = 0.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 3250;
    private static final double MAX_INTEGRAL = 500.0; // anti-windup clamp
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

    // PID variables
    //private double currentVelocity, currentRPM, error, lastError, integralSum, derivative, output, deltaTime;

    // Poses
    private final Pose startPose = new Pose(124.5, 119.6, Math.toRadians(36));
    private final Pose scorePose = new Pose(101, 100, Math.toRadians(36));
    private final Pose shakePose = new Pose(101, 93, Math.toRadians(36));
    private final Pose set1frontPose = new Pose(90, 75, Math.toRadians(0));
    private final Pose set1controlPoint = new Pose(60, 90);
    private final Pose set1pickPose = new Pose(128, 80, Math.toRadians(0));
    private final Pose set2frontPose = new Pose(90, 39, Math.toRadians(0));
    private final Pose set2controlPoint = new Pose(70, 84);
    private final Pose set2pickPose = new Pose(137, 46, Math.toRadians(0));
    private final Pose set2OutPose = new Pose(90, 40, Math.toRadians(0));
    private final Pose moveOutPose = new Pose(120, 85, Math.toRadians(0));

    // Paths
    private Path scorePreload;
    private PathChain frontSet1, set2Out, pickupSet1, scoreSet1, frontSet2, shake, shakeBack, pickupSet2, scoreSet2, moveOut;

    private Pose computeRetreatPose(Pose origin, double distanceInches) {
        double heading = origin.getHeading();
        double dx = -distanceInches * Math.cos(heading);
        double dy = -distanceInches * Math.sin(heading);
        return new Pose(origin.getX() + dx, origin.getY() + dy, heading);
    }


    public void buildPaths() {
        //Score Preload (backwards 30 inches)
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45));

        //Drive in front of set 1
        frontSet1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, set1controlPoint, set1frontPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), set1frontPose.getHeading())
                .build();

        //Pickup set 1
        pickupSet1 = follower.pathBuilder()
                .addPath(new BezierLine(set1frontPose, set1pickPose))
                .setConstantHeadingInterpolation(set1pickPose.getHeading())
                .build();

        //Score set 1
        scoreSet1 = follower.pathBuilder()
                .addPath(new BezierLine(set1pickPose, scorePose))
                .setLinearHeadingInterpolation(set1pickPose.getHeading(), scorePose.getHeading())
                .build();

        shake = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, shakePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        shakeBack = follower.pathBuilder()
                .addPath(new BezierLine(shakePose, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        //Drive in front of set 2
        frontSet2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, set2controlPoint, set2frontPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), set2frontPose.getHeading())
                .build();

        //Pickup set 2
        pickupSet2 = follower.pathBuilder()
                .addPath(new BezierLine(set2frontPose, set2pickPose))
                .setConstantHeadingInterpolation(set2pickPose.getHeading())
                .build();

        //Back set 2
        set2Out = follower.pathBuilder()
                .addPath(new BezierLine(set2pickPose, set2OutPose))
                .setLinearHeadingInterpolation(set2pickPose.getHeading(), scorePose.getHeading())
                .build();

        //Score set 2
        scoreSet2 = follower.pathBuilder()
                .addPath(new BezierLine(set2OutPose, scorePose))
                .setLinearHeadingInterpolation(set2pickPose.getHeading(), scorePose.getHeading())
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

    public void updateShooterPID() {
        currentVelocity = shooterMotor.getVelocity();
        currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;

        error = TARGET_RPM - currentRPM;

        double currentTime = timer.seconds();
        deltaTime = currentTime - lastTimestamp;
        lastTimestamp = currentTime;

        if (deltaTime <= 0) return; // protect against divide-by-zero

        // Integral with anti-windup
        integralSum += error * deltaTime;
        integralSum = Math.max(Math.min(integralSum, MAX_INTEGRAL), -MAX_INTEGRAL);

        // Derivative
        derivative = (error - lastError) / deltaTime;

        // PID output
        output = (SHOOTER_KP * error) + (SHOOTER_KI * integralSum) + (SHOOTER_KD * derivative);

        // Clamp to motor power range
        output = Math.max(Math.min(output, 1.0), -1.0);

        shooterMotor.setPower(output);

        lastError = error;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                follower.followPath(scorePreload);
                telemetry.addLine("Moving back to score preload");
                waitTime = 1000;
                break;
            case 2: case 3: case 4:
                shootBall();
                telemetry.addLine("Scoring preload balls");
                waitTime = 100;
                break;
            case 5:
                waitTime = 100;
                break;
            case 6:
                rubberIntakeMotor.setPower(1);
                follower.followPath(frontSet1);
                telemetry.addLine("Driving to the front of set 1");
                waitTime = 1500;
                break;
            case 7:
                follower.setMaxPower(0.7);
                follower.followPath(pickupSet1);
                telemetry.addLine("Picking up set 1");
                waitTime = 3000;
                break;
            case 8:
                follower.setMaxPower(1);
                follower.followPath(scoreSet1);
                follower.followPath(shake);
                follower.followPath(shakeBack);
                follower.followPath(shake);
                follower.followPath(shakeBack);
                telemetry.addLine("Driving to score set 1");
                waitTime = 2000;
                break;
            case 9: case 10: case 11:
                shootBall();
                waitTime = 100;
                break;
            case 12:
                rubberIntakeMotor.setPower(1);
                follower.followPath(frontSet2);
                telemetry.addLine("Driving to the front of set 2");
                waitTime = 1500;
                break;
            case 13:
                follower.setMaxPower(1);
                follower.followPath(pickupSet2);
                telemetry.addLine("Picking up set 2");
                waitTime = 2000;
                break;
            case 14:
                follower.setMaxPower(1);
                follower.followPath(set2Out);
                follower.followPath(scoreSet2);
                telemetry.addLine("Driving to score set 2");
                waitTime = 2000;
                break;
            case 15: case 16: case 17:
                shootBall();
                waitTime = 100;
                break;
            case 18:
                follower.followPath(moveOut);
                telemetry.addLine("Moving out");
                waitTime = 1000;
                break;
            case 19:
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
