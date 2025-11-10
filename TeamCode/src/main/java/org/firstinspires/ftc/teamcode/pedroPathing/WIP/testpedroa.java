package org.firstinspires.ftc.teamcode.pedroPathing.WIP;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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
@Autonomous(name = "PARAPARAPA", group = "Concept")
public class testpedroa extends OpMode {
    private double shooterPower = 0.0;
    private AprilTagDetection lastDetection = null;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private Timer pathTimer;
    //cat11red!
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    private String color1 = "none";
    private String color2 = "none";
    private String color3 = "none";
    private String obelisk = "none";

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


    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (!currentDetections.isEmpty()) {
            lastDetection = currentDetections.get(0); // Save the first detection
        }

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


    private int pathState;


    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose scorePose = new Pose(1, 0, Math.toRadians(90));
    private Path scorePreload;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        if (lastDetection == null) {
            telemetry.addLine("No AprilTag detected yet; skipping shooter calc");
            return;
        }

        double xMeters = lastDetection.ftcPose.x * 0.0254;
        double yMeters = lastDetection.ftcPose.y * 0.0254;
        double distanceMeters = Math.sqrt(xMeters * xMeters + yMeters * yMeters);
        double launchAngleDeg = 30;
        double shooterHeight = 11 * 0.0254;
        double targetHeight = 45 * 0.0254;
        double wheelRadius = 0.092075;
        double gearRatio = 1.0;
        double fudgeFactor = 1.0;

        if (pathState == 0) {
            if (!follower.isBusy()) {
                shooterPower = ShooterCalculator.getShooterMotorPower(
                        distanceMeters,
                        launchAngleDeg,
                        shooterHeight,
                        targetHeight,
                        wheelRadius,
                        gearRatio,
                        fudgeFactor
                );
                shooterMotor.setPower(-shooterPower);
                follower.followPath(scorePreload);
                setPathState(1);
            }
        } else {
            telemetry.addLine("Auto Done");
        }
    }



    @Override
    public void loop() {

        telemetryAprilTag();
        follower.update();
        autonomousPathUpdate();

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

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        pathTimer = new Timer();
        pathTimer.resetTimer();

        setPathState(0);

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


    }
}
