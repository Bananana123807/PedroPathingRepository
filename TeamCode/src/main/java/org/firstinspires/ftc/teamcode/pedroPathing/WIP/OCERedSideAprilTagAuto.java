package org.firstinspires.ftc.teamcode.pedroPathing.WIP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Disabled
@Autonomous(name = "Concept: AprilTag", group = "Concept")
public class OCERedSideAprilTagAuto extends OpMode {
    private double shooterPower = 0.0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private String color1 = "none";
    private String color2 = "none";
    private String color3 = "none";
    private String obelisk = "none";

    // TODO: Declare your shooter motor here
    private DcMotor shooterMotor = null;
    public static class ShooterCalculator {

        static final double G = 9.81; // Gravity (m/s^2)
        static final double RPM_FREE = 6000.0; // GoBilda motor free speed
        static final double OMEGA_FREE = (RPM_FREE / 60.0) * 2.0 * Math.PI; // rad/s

        public static double getShooterMotorPower(
                double distanceMeters,
                double launchAngleDeg,
                double shooterHeight,
                double targetHeight,
                double wheelRadius,
                double gearRatio,
                double fudgeFactor
        ) {
            double thetaRad = Math.toRadians(launchAngleDeg);
            double deltaH = shooterHeight - targetHeight;

            double denominator = 2.0 * Math.pow(Math.cos(thetaRad), 2.0) *
                    (distanceMeters * Math.tan(thetaRad) + deltaH);

            if (denominator <= 0) {
                return 0.0; // Invalid shot
            }

            double v = Math.sqrt((G * Math.pow(distanceMeters, 2.0)) / denominator);

            double omegaWheel = v / wheelRadius;
            double omegaMotor = gearRatio * omegaWheel;

            double power = (omegaMotor / OMEGA_FREE) * fudgeFactor;

            return Math.min(1.0, Math.max(0.0, power));
        }
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Camera"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Only calculate shooter power for the first detection (if any)
        boolean powerSet = false;

        for (AprilTagDetection detection : currentDetections) {
            switch (detection.id) {
                case 20:
                    obelisk = "Blue";
                    break;
                case 21:
                    color1 = "green";
                    color2 = "purple";
                    color3 = "purple";
                    obelisk = "Communism Was Kinda Tight";
                    break;
                case 22:
                    color1 = "purple";
                    color2 = "green";
                    color3 = "purple";
                    obelisk = "Communism Was Kinda Tight";
                    break;
                case 23:
                    color1 = "purple";
                    color2 = "purple";
                    color3 = "green";
                    obelisk = "Communism Was Kinda Tight";
                    break;
                case 24:
                    obelisk = "Red";
                    break;
                default:
                    obelisk = "Communism Was Kinda Tight";
                    color1 = "unknown";
                    color2 = "unknown";
                    color3 = "unknown";
                    break;
            }

            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

            // Calculate and set shooter power only once, for the first detection
            if (!powerSet) {
                double xMeters = detection.ftcPose.x * 0.0254;
                double yMeters = detection.ftcPose.y * 0.0254;
                double distanceMeters = Math.sqrt(xMeters * xMeters + yMeters * yMeters);
                double launchAngleDeg = 31.680513712215;
                double shooterHeight = 11 * 0.0254;
                double targetHeight = 45 * 0.0254;
                double wheelRadius = 0.092075;
                double gearRatio = 1.0;
                double fudgeFactor = 0;

                shooterPower = ShooterCalculator.getShooterMotorPower(
                        distanceMeters,
                        launchAngleDeg,
                        shooterHeight,
                        targetHeight,
                        wheelRadius,
                        gearRatio,
                        fudgeFactor
                );

                powerSet = true;
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        telemetry.addData("First Color", color1);
        telemetry.addData("Second Color", color2);
        telemetry.addData("Third Color", color3);

        telemetry.addData("Obelisk Color:", obelisk);
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(120.358, 120.944, Math.toRadians(45)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(88.706, 87.924, Math.toRadians(45));
    // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.706, 87.924),
                                new Pose(74.638, 74.247),
                                new Pose(120.749, 84.407)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.749, 84.407),
                                new Pose(74.638, 74.052),
                                new Pose(88.706, 87.924)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.706, 87.924),
                                new Pose(74.247, 50.801),
                                new Pose(120.944, 59.788)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.944, 59.788),
                                new Pose(74.247, 50.801),
                                new Pose(88.706, 87.924)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.706, 87.924),
                                new Pose(73.661, 26.182),
                                new Pose(120.749, 35.365)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.749, 35.365),
                                new Pose(73.465, 26.182),
                                new Pose(88.706, 87.924)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                follower.followPath(grabPickup1);
                setPathState(2);
                shooterMotor.setPower(shooterPower);
                break;
            case 2:
                follower.followPath(scorePickup1);
                setPathState(3);
                break;
            case 3:
                follower.followPath(grabPickup2);
                setPathState(4);
                break;
            case 4:
                follower.followPath(scorePickup2);
                setPathState(5);
                break;
            case 5:
                follower.followPath(grabPickup3);
                setPathState(6);
                break;
            case 6:
                follower.followPath(scorePickup3);
                setPathState(7);
                break;
        }
    }

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        initAprilTag();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        telemetryAprilTag();

        telemetry.update();

        visionPortal.resumeStreaming();

        visionPortal.close();

    }
     public void runOPMode(){
         shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
    }
}
