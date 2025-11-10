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
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
        telemetry.addData("id20 String", id20.toString());
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificID(24);
        aprilTagWebcam.displayDetectionTelemetry(id24);
        telemetry.addData("id24 String", id24.toString());
    }
}
