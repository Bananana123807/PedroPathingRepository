package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class AprilTag extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        double posX;
        double posY;
        double posZ;
        double roll;
        double pitch;
        double yaw;
        int tagID;

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                //.setCamera(WebcamName.class, "Webcam 1")
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if (!tagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                tagID = tag.id;

                telemetry.addData("Tag ID: ", tagID);

                // Access position (x, y, z) and orientation (roll, pitch, yaw)
                posX = tag.ftcPose.x;
                posY = tag.ftcPose.y;
                posZ = tag.ftcPose.z;
                roll = tag.ftcPose.roll;
                pitch = tag.ftcPose.pitch;
                yaw = tag.ftcPose.yaw;

                // Use these variables for whatever you need, e.g., telemetry
                telemetry.addData("x", posX);
                telemetry.addData("y", posY);
                telemetry.addData("z", posZ);
                telemetry.addData("roll", roll);
                telemetry.addData("pitch", pitch);
                telemetry.addData("yaw", yaw);
                if (tagID == 24) { //for Red
                    telemetry.addData("Color", "Red");
                } else if (tagID == 20) { //for Blue
                    telemetry.addData("Color", "Blue");
                } else {
                    telemetry.addData("Communism was ", "kinda tight");
                }



            } else {
                telemetry.addLine("No tag seen.");
            }

            telemetry.update();
        }
    }
}

