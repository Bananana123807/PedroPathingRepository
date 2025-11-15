package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueSideCloseAuto", group = "Over-caffeinated")
public class BlueSideCloseAuto extends OpMode {

    // PID constants
    private static final double SHOOTER_KP = 0.0007;
    private static final double SHOOTER_KI = 0.0005;
    private static final double SHOOTER_KD = 0.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 3100;
    private static final double MAX_INTEGRAL = 500.0;

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
    private PredominantColorProcessor colorSensor;

    // Pathing
    private Follower follower;
    private Timer pathTimer, ballTimer, pickTimer;
    private int pathState = 0;
    private int counter = 0;
    private boolean isShooting = false;
    private double waitTime = 0;
    private String ballColor = "";
    private int ballNum = 0;

    // -------- BLUE SIDE MIRRORED FIELD POSES --------
    private final Pose startPose = new Pose(19.5, 119.6, Math.toRadians(144));
    private final Pose retreatPose = computeRetreatPose(startPose, 30);
    private final Pose behindBallsPose = new Pose(64, 77.5, Math.toRadians(180));
    private final Pose ball1 = new Pose(39, 77.5, Math.toRadians(180));
    private final Pose ball2 = new Pose(31, 77.5, Math.toRadians(180));
    private final Pose ballSweepEndPose = new Pose(10, 77.5, Math.toRadians(180));
    private final Pose scorePose = new Pose(44, 99, Math.toRadians(144));
    private final Pose scoreControlPointPose = new Pose(30, 102);
    private final Pose exitStartPose = new Pose(44, 99, Math.toRadians(180));
    private final Pose exitPose = new Pose(24, 80, Math.toRadians(180));

    // Path objects
    private Path scorePreload, driveToBalls, toBall1, toBall2, ballSweep;
    private PathChain scoreAllBalls, moveOut;


    // Compute backwards retreat relative to heading
    private Pose computeRetreatPose(Pose origin, double distanceInches) {
        double heading = origin.getHeading();
        double dx = -distanceInches * Math.cos(heading);
        double dy = -distanceInches * Math.sin(heading);
        return new Pose(origin.getX() + dx, origin.getY() + dy, heading);
    }

    // Build all driving paths
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, retreatPose));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(144));

        driveToBalls = new Path(new BezierLine(retreatPose, behindBallsPose));
        driveToBalls.setConstantHeadingInterpolation(Math.toRadians(180));

        toBall1 = new Path(new BezierLine(behindBallsPose, ball1));
        toBall1.setConstantHeadingInterpolation(Math.toRadians(180));

        toBall2 = new Path(new BezierLine(ball1, ball2));
        toBall2.setConstantHeadingInterpolation(Math.toRadians(180));

        ballSweep = new Path(new BezierLine(behindBallsPose, ballSweepEndPose));
        ballSweep.setConstantHeadingInterpolation(Math.toRadians(180));

        scoreAllBalls = follower.pathBuilder()
                .addPath(new BezierCurve(ballSweepEndPose, scoreControlPointPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();

        moveOut = follower.pathBuilder()
                .addPath(new BezierLine(exitStartPose, exitPose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
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
                follower.setMaxPower(1);
                updateShooterPID();
                follower.followPath(scorePreload);
                telemetry.addLine("Backing up to shoot preload");
                waitTime = 2000;
                break;
            case 2: case 3: case 4:
                shootBall();
                waitTime = 100;
                break;
            case 5:
                follower.followPath(driveToBalls);
                telemetry.addLine("Driving to ball sweep start");
                waitTime = 1500;
                break;
            case 6:
                follower.setMaxPower(0.5);
                follower.followPath(toBall1);
                telemetry.addLine("Driving to Ball 1");
                waitTime = 2000;
                break;
            case 7:
                follower.followPath(toBall2);
                telemetry.addLine("Driving to Ball 2");
                waitTime = 2000;
                break;
            case 8:
                follower.followPath(ballSweep);
                telemetry.addLine("Sweeping through balls");
                waitTime = 3000;
                break;
            case 9:
                follower.setMaxPower(1);
                follower.followPath(scoreAllBalls);
                telemetry.addLine("Returning to score");
                waitTime = 1500;
                break;
            case 10: case 11: case 12:
                shootBall();
                waitTime = 100;
                break;
            case 13:
                follower.followPath(moveOut);
                telemetry.addLine("Moving out");
                waitTime = 1000;
                break;
            case 14:
                gate.setPower(0);
                shooterMotor.setPower(0);
                noodleIntake.setPower(0);
                telemetry.addLine("Auto Complete");
                break;
        }
    }


    @Override
    public void loop() {
        updateShooterPID();
        follower.update();

        double elapsedTime = pathTimer.getElapsedTime();
        double elapsedPickTime = pickTimer.getElapsedTime();

        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        if (result == null) {
            telemetry.addLine("Waiting for camera...");
            return;
        }

        if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
            ballColor = "Green";
        } else if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
            ballColor = "Purple";
        } else {
            ballColor = "None";
        }

        if ((ballColor.equals("Purple") || ballColor.equals("Green"))) {
            if(elapsedPickTime > 50) {
                server.setPosition(ACTIVE_GATE_POSITION);
                ballColor = "No Ball Detected";
            }
        } else {
            server.setPosition(DEFAULT_GATE_POSITION);
            pickTimer.resetTimer();
        }

        if (isShooting) shootBall();

        if (elapsedTime >= waitTime && !isShooting) {
            counter += 1;
            setPathState(counter);
            autonomousPathUpdate();
        }

        telemetry.addData("Color Detected", ballColor);
        telemetry.addData("Best Match", result.closestSwatch);
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

        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE
                )
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .build();

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
