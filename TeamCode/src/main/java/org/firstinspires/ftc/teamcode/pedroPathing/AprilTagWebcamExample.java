package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagWebcamExample extends OpMode {

    AprilTagDistanceAndID aprilTagWebcam = new AprilTagDistanceAndID();

    @Override
    public void init(){
        aprilTagWebcam.init(hardwareMap, telemetry);
    }
    @Override
    public void loop(){
        aprilTagWebcam.update();

        //Blue Side AprilTag ID
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);

        //AprilTag ID for Motifs
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(id21);

        AprilTagDetection id22 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(id22);

        AprilTagDetection id23 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(id23);
        //

        //Red Side AprilTag ID
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificID(24);
        aprilTagWebcam.displayDetectionTelemetry(id24);
    }
}
