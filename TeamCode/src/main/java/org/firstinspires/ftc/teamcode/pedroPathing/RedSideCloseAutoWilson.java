package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import com.qualcomm.robotcore.hardware.IMU;


@Autonomous(name = "RedSideCloseAutoWilson", group = "Over-caffeinated")
public class RedSideCloseAutoWilson extends OpMode {

    private Follower follower;
    private Timer timer, pickTimer, shootTimer;
    private DcMotor shooterMotor, noodleIntake;

    private IMU imu;
    private CRServo gate;
    private Servo server;
    private int state = 0;
    private boolean isShooting = false;
    private int ballCount = 0;

    private PredominantColorProcessor colorSensor;
    private String ballColor = "";

    private final Pose startPose = new Pose(122.941, 123.510, Math.toRadians(36));

    private final Pose shoot1Pose = new Pose(94.198, 102.451, Math.toRadians(36));
    private final Pose afterShoot1Pose = new Pose(83.66798, 83.95256, Math.toRadians(0));
    private final Pose ball1Pose = new Pose(109.2806, 84.23715, Math.toRadians(1));
    private final Pose ball2Pose = new Pose(115.82608, 84.2371, Math.toRadians(0));
    private final Pose ball3Pose = new Pose(130.9090, 83.66798, Math.toRadians(-2));
    private final Pose shoot2Pose = new Pose(101.88142, 101.31225, Math.toRadians(40));
    private final Pose afterShoot2Pose = new Pose(101.881422, 70.00790, Math.toRadians(0));

    private PathChain moveToShoot1, afterShoot1, ball1Path, ball2Path, ball3Path, moveToShoot2, afterShoot2;

    public void buildPaths() {
        moveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1Pose))
                .setConstantHeadingInterpolation(shoot1Pose.getHeading()) // use getter
                .build();

        afterShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, afterShoot1Pose))
                .setConstantHeadingInterpolation(afterShoot1Pose.getHeading())
                .build();

        ball1Path = follower.pathBuilder()
                .addPath(new BezierLine(afterShoot1Pose, ball1Pose))
                .setConstantHeadingInterpolation(ball1Pose.getHeading())
                .build();

        ball2Path = follower.pathBuilder()
                .addPath(new BezierLine(ball1Pose, ball2Pose))
                .setConstantHeadingInterpolation(ball2Pose.getHeading())
                .build();

        ball3Path = follower.pathBuilder()
                .addPath(new BezierLine(ball2Pose, ball3Pose))
                .setConstantHeadingInterpolation(ball3Pose.getHeading())
                .build();

        moveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(ball3Pose, shoot2Pose))
                .setConstantHeadingInterpolation(shoot2Pose.getHeading()) // matches your visualizer
                .build();

        afterShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, afterShoot2Pose))
                .setConstantHeadingInterpolation(afterShoot2Pose.getHeading())
                .build();
    }


    public void shootBall() {
        if (!isShooting) {
            gate.setPower(1);
            shootTimer.resetTimer();
            isShooting = true;
        }

        double elapsed = shootTimer.getElapsedTime();

        if (elapsed > 1000 && elapsed < 1800) {
            gate.setPower(-1);
        } else if (elapsed >= 1800) {
            gate.setPower(0);
            isShooting = false;
            ballCount++;
            shootTimer.resetTimer();
        }
    }

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        gate = hardwareMap.get(CRServo.class, "gate");
        server = hardwareMap.get(Servo.class, "server");
        noodleIntake = hardwareMap.get(DcMotor.class, "intake");
        noodleIntake.setPower(0.7);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));

        imu.resetYaw();

        timer = new Timer();
        pickTimer = new Timer();
        shootTimer = new Timer();

        buildPaths();

        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
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
    }

    @Override
    public void start() {
        timer.resetTimer();
        shooterMotor.setPower(-0.55);
    }

    @Override
    public void loop() {
        follower.update();
        double elapsed = timer.getElapsedTime();

        // Vision-based intake logic
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        if (result != null) {
            if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
                ballColor = "Green";
            } else if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
                ballColor = "Purple";
            }

            if ((ballColor.equals("Green") || ballColor.equals("Purple")) && pickTimer.getElapsedTime() > 500) {
                server.setPosition(0.85);
                pickTimer.resetTimer();
            } else {
                server.setPosition(0.4);
            }
        }

        switch (state) {
            case 0:
                follower.followPath(moveToShoot1);
                timer.resetTimer();
                state++;
                break;
            case 1:
                if (elapsed > 4000) shootBall();
                if (ballCount >= 3) {
                    timer.resetTimer();
                    state++;
                }
                break;
            case 2:
                follower.followPath(afterShoot1);
                timer.resetTimer();
                state++;
                break;
            case 3:
                follower.followPath(ball1Path);
                timer.resetTimer();
                state++;
                break;
            case 4:
                if (elapsed > 3000) {
                    timer.resetTimer();
                    state++;
                }
                break;
            case 5:
                follower.followPath(ball2Path);
                timer.resetTimer();
                state++;
                break;
            case 6:
                if (elapsed > 3000) {
                    timer.resetTimer();
                    state++;
                }
                break;
            case 7:
                follower.followPath(ball3Path);
                timer.resetTimer();
                state++;
                break;
            case 8:
                if (elapsed > 3000) {
                    timer.resetTimer();
                    state++;
                }
                break;
            case 9:
                follower.followPath(moveToShoot2);
                timer.resetTimer();
                state++;
                break;
            case 10:
                if (elapsed > 2000) shootBall();
                if (ballCount >= 6) {
                    timer.resetTimer();
                    state++;
                }
                break;
            case 11:
                follower.followPath(afterShoot2);
                state++;
                break;
            case 12:
                shooterMotor.setPower(0);
                gate.setPower(0);
                telemetry.addLine("Auto Complete");
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Ball Color", ballColor);
        telemetry.update();
    }
}
