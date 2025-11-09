//package org.firstinspires.ftc.teamcode.pedroPathing.WIP;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class AprilTagIDandDistance {
//    private AprilTagProcessor aprilTagProcessor;
//    private VisionPortal visionPortal;
//    private List<AprilTagDetection> detectedTags = new ArrayList<>();
//    private Telemetry telemetry;
//
//    public void init(HardwareMap hwMap, Telemetry telemetry) {
//        this.telemetry = telemetry;
//
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES);
//    }
//}
