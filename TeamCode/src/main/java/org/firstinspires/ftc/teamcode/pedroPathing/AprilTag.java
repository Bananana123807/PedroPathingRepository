package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "APRIL_TAG_OB", group = "Concept")
public class AprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the AprilTag processor with drawing options
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Build the vision portal with the camera and processor
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480)); // Optional: set resolution

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Camera")); // Use your configured webcam name
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK); // Use phone's back camera
        }

        visionPortal = builder.build();

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (!aprilTagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(0); // Grab first detected tag
                int tagID = tag.id;

                double posX = tag.ftcPose.x;
                double posY = tag.ftcPose.y;
                double posZ = tag.ftcPose.z;
                double roll = tag.ftcPose.roll;
                double pitch = tag.ftcPose.pitch;
                double yaw = tag.ftcPose.yaw;

                telemetry.addData("Tag ID", tagID);
                telemetry.addData("x", posX);
                telemetry.addData("y", posY);
                telemetry.addData("z", posZ);
                telemetry.addData("roll", roll);
                telemetry.addData("pitch", pitch);
                telemetry.addData("yaw", yaw);

                if (tagID == 24) {
                    telemetry.addData("Color", "Red");
                } else if (tagID == 20) {
                    telemetry.addData("Color", "Blue");
                } else {
                    telemetry.addData("Communism was", "kinda tight");
                }
            } else {
                telemetry.addLine("Communism was kinda tight");
            }

            telemetry.update();
        }
    }
}
