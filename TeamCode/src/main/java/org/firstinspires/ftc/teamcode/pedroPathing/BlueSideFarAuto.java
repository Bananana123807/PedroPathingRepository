package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "BlueSideFarAuto", group = "Over-caffeinated")
public class BlueSideFarAuto extends OpMode {

    // PID constants
    private static final double SHOOTER_KP = 0.0007;
    private static final double SHOOTER_KI = 0.0005;
    private static final double SHOOTER_KD = 0.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 5160;
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

    // Mirrored Poses (Blue Side)
    private final Pose startPose = new Pose(56, 3, Math.toRadians(90));
    private final Pose scorePose = new Pose(60.048, 14.2292, Math.toRadians(114));
    private final Pose outPose = new Pose(9, 23, Math.toRadians(180));

    // Paths
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

            case 2:
            case 3:
            case 4:
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
                        PredominantColorProcessor.Swatch.YELLOW,
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

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }
}

//@Autonomous(name = "BlueSideFarAuto", group = "Over-caffeinated")
//public class BlueSideFarAuto extends OpMode {
//
//    private static final double SHOOTER_KP = 0.0007;
//    private static final double SHOOTER_KI = 0.0005;
//    private static final double SHOOTER_KD = 0.0;
//    private static final double TICKS_PER_REV = 28.0;
//    private static final double TARGET_RPM = 5160;
//    private static final double MAX_INTEGRAL = 500.0;
//
//    private ElapsedTime timer = new ElapsedTime();
//    private double currentRPM = 0, error = 0, lastError = 0, integralSum = 0, lastTimestamp = 0;
//    private static final double DEFAULT_GATE_POSITION = 0.4;
//    private static final double ACTIVE_GATE_POSITION = 0.85;
//
//    private DcMotorEx shooterMotor;
//    private CRServo gate;
//    private DcMotor noodleIntake;
//    private Servo server;
//    private PredominantColorProcessor colorSensor;
//
//    private Follower follower;
//    private Timer pathTimer, ballTimer, pickTimer;
//    private int pathState = 0;
//    private int ballNum = 0;
//    private boolean isShooting = false;
//    private double waitTime = 0;
//    private String ballColor = "";
//
//    private final Pose startPose = new Pose(88, 145, Math.toRadians(90));
//    private final Pose scorePose = new Pose(83.952, 133.7708, Math.toRadians(120));
//    private final Pose outPose = new Pose(135, 125, Math.toRadians(180));
//
//    private Path scorePath, outPath;
//
//    public void buildPaths() {
//        scorePath = new Path(new BezierLine(startPose, scorePose));
//        scorePath.setConstantHeadingInterpolation(Math.toRadians(291));
//
//        outPath = new Path(new BezierLine(scorePose, outPose));
//        outPath.setConstantHeadingInterpolation(Math.toRadians(180));
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    public void shootBall() {
//        if (!isShooting) {
//            ballNum++;
//            gate.setPower(1);
//            ballTimer.resetTimer();
//            isShooting = true;
//        }
//        double elapsed = ballTimer.getElapsedTime();
//        if (elapsed >= 900 && elapsed < 1400) gate.setPower(-1);
//        else if (elapsed >= 1300) {
//            gate.setPower(0);
//            isShooting = false;
//        }
//    }
//
//    public void updateShooterPID() {
//        double velocity = shooterMotor.getVelocity();
//        currentRPM = (velocity / TICKS_PER_REV) * 60.0;
//        error = TARGET_RPM - currentRPM;
//        double currTime = timer.seconds();
//        double dt = currTime - lastTimestamp;
//        lastTimestamp = currTime;
//        if (dt <= 0) return;
//        integralSum += error * dt;
//        integralSum = Math.max(Math.min(integralSum, MAX_INTEGRAL), -MAX_INTEGRAL);
//        double derivative = (error - lastError) / dt;
//        lastError = error;
//        double output = (SHOOTER_KP * error) + (SHOOTER_KI * integralSum) + (SHOOTER_KD * derivative);
//        output = Math.max(Math.min(output, 1.0), -1.0);
//        shooterMotor.setPower(output);
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 1:
//                follower.followPath(scorePath);
//                waitTime = 3000;
//                break;
//            case 2: case 3: case 4:
//                shootBall();
//                waitTime = 3000;
//                break;
//            case 5:
//                follower.followPath(outPath);
//                waitTime = 1500;
//                break;
//        }
//    }
//
//    @Override
//    public void loop() {
//        updateShooterPID();
//        follower.update();
//
//        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
//        if (result != null) {
//            switch (result.closestSwatch) {
//                case ARTIFACT_GREEN:  ballColor = "Green";  break;
//                case ARTIFACT_PURPLE: ballColor = "Purple"; break;
//                default: ballColor = "None";
//            }
//        }
//
//        if (ballColor.equals("Purple") || ballColor.equals("Green")) {
//            if (pickTimer.getElapsedTime() > 50) {
//                server.setPosition(ACTIVE_GATE_POSITION);
//                ballColor = "Reset";
//            }
//        } else {
//            server.setPosition(DEFAULT_GATE_POSITION);
//            pickTimer.resetTimer();
//        }
//
//        if (isShooting) shootBall();
//
//        if (pathTimer.getElapsedTime() >= waitTime && !isShooting) {
//            setPathState(pathState + 1);
//            autonomousPathUpdate();
//        }
//
//        telemetry.addData("Color", ballColor);
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("Pose", follower.getPose());
//        telemetry.addData("Shooter RPM", currentRPM);
//        telemetry.addData("Ball Fired Count", ballNum);
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        pathTimer.resetTimer();
//        noodleIntake.setPower(-0.67);
//        setPathState(1);
//    }
//
//    @Override
//    public void init() {
//        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
//        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        gate = hardwareMap.get(CRServo.class, "gate");
//        noodleIntake = hardwareMap.get(DcMotor.class, "intake");
//        server = hardwareMap.get(Servo.class, "server");
//
//        colorSensor = new PredominantColorProcessor.Builder()
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
//                .setSwatches(
//                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
//                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
//                        PredominantColorProcessor.Swatch.YELLOW,
//                        PredominantColorProcessor.Swatch.BLACK,
//                        PredominantColorProcessor.Swatch.WHITE
//                )
//                .build();
//
//        VisionPortal portal = new VisionPortal.Builder()
//                .addProcessor(colorSensor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
//                .setCameraResolution(new Size(320, 240))
//                .build();
//
//        pathTimer = new Timer();
//        ballTimer = new Timer();
//        pickTimer = new Timer();
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//    }
//}

