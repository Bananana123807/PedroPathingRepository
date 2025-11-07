package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;
import com.pedropathing.follower.Follower;
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
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Autonomous(name = "RedSideCloseAuto2", group = "Over-caffeinated")
public class RedSideCloseAuto extends OpMode {

    private double shooterPower = -0.55;
    private double gatePower = -1;
    int ballNum = 0;
    private Follower follower;
    private Timer pathTimer, ballTimer, pickTimer;
    private DcMotor shooterMotor = null;
    private CRServo gate = null;
    private int pathState;
    private int counter = 0;
    private final Pose startPose = new Pose(0, 0);
    private final Pose scorePose = new Pose(-30, 0);
    private final Pose setOne = new Pose(-40, -10);
    private final Pose ball1Pose = new Pose(-40, -15);
    private final Pose ball2Pose = new Pose(-40, -20);
    private final Pose ball3Pose = new Pose(-40, -25);
    private final Pose moveOutPose = new Pose(-30, -15);
    private Path scorePreload;
    private PathChain setOnePath, ball1, ball2, ball3, moveOut, scoreSet1;
    private double waitTime = 0;
    private boolean isShooting = false;
    private String ballColor = "";
    DcMotor noodleIntake;
    Servo server;
    private PredominantColorProcessor colorSensor;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));

        setOnePath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, setOne))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-36))
                .build();

        ball1 = follower.pathBuilder()
                .addPath(new BezierLine(setOne, ball1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-36))
                .build();

        ball2 = follower.pathBuilder()
                .addPath(new BezierLine(ball1Pose, ball2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-36))
                .build();

        ball3 = follower.pathBuilder()
                .addPath(new BezierLine(ball2Pose, ball3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-36))
                .build();

        scoreSet1 = follower.pathBuilder()
                .addPath(new BezierLine(ball3Pose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(0))
                .build();

        moveOut = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, moveOutPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
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
            gate.setPower(gatePower);
        } else if (elapsed >= 1500) {
            gate.setPower(0);
            isShooting = false;
        }
    }

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 1:
                shooterMotor.setPower(shooterPower);
                follower.followPath(scorePreload);
                telemetry.addLine("Moving Back");
                waitTime = 2000;
                break;
            case 2:
            case 3:
            case 4:
            case 10:
            case 11:
            case 12:
                shootBall();
                waitTime = 100;
                break;
            case 5:
                follower.followPath(setOnePath);
                telemetry.addLine("PickupPose");
                waitTime = 1500;
                break;
            case 6:
                follower.followPath(ball1);
                telemetry.addLine("Picking ball 1");
                waitTime = 1500;
                break;
            case 7:
                follower.followPath(ball2);
                telemetry.addLine("Picking ball 2");
                waitTime = 1500;
                break;
            case 8:
                follower.followPath(ball3);
                telemetry.addLine("Picking ball 3");
                waitTime = 1500;
                break;
            case 9:
                follower.followPath(scoreSet1);
                telemetry.addLine("Scoring");
                waitTime = 1500;
                break;
            case 13:
                follower.followPath(moveOut);
                telemetry.addLine("Moving Out");
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
        follower.update();

        double elapsedTime = pathTimer.getElapsedTime();
        double elapsedPickTime = pickTimer.getElapsedTime();
        telemetry.addLine("Preview on/off: 3 dots, Camera Stream\n");

        PredominantColorProcessor.Result result = colorSensor.getAnalysis();

        if (result == null) {
            telemetry.addLine("Waiting for camera...");
            return;
        }

        if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
            ballColor = "Green";
            telemetry.addData("Color Detected:", ballColor);
        }

        if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
            ballColor = "Purple";
            telemetry.addData("Color Detected:", ballColor);
        }

        if (ballColor.equals("Green") || ballColor.equals("Purple")){
            if (elapsedPickTime > 500){
                server.setPosition(0.85);
                ballColor = "No Ball Detected";
                pickTimer.resetTimer();
            }
        } else {
            server.setPosition(0.4);
        }

        if (isShooting) {
            shootBall();
        }

        if (elapsedTime >= waitTime && !isShooting) {
            counter += 1;
            setPathState(counter);
            autonomousPathUpdate();
        }

        telemetry.addData("Best Match", result.closestSwatch);
        telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)",
                result.RGB[0], result.RGB[1], result.RGB[2]));
        telemetry.addLine(String.format("HSV   (%3d, %3d, %3d)",
                result.HSV[0], result.HSV[1], result.HSV[2]));
        telemetry.addLine(String.format("YCrCb (%3d, %3d, %3d)",
                result.YCrCb[0], result.YCrCb[1], result.YCrCb[2]));
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.addData("Shooting Ball Number: ", ballNum);
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        noodleIntake.setPower(-0.67);
    }


    @Override
    public void init(){

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        gate = hardwareMap.get(CRServo.class, "gate");
        noodleIntake = hardwareMap.get(DcMotor.class, "intake");
        server = hardwareMap.get(Servo.class, "server");

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

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        pathTimer = new Timer();
        pathTimer.resetTimer();

        ballTimer = new Timer();
        ballTimer.resetTimer();

        pickTimer = new Timer();
        pickTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        Constants.localizerConstants.imu.resetYaw();
        telemetry.update();
    }
}