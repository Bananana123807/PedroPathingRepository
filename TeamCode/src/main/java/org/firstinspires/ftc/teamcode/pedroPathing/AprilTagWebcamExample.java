package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous(name="AprilDistanceWebcam", group="OCE")
public class AprilTagWebcamExample extends OpMode {
    AprilTagDistanceAndID aprilTagWebcam = new AprilTagDistanceAndID();
    DcMotorEx shooterMotor;
    CRServo gate;
    private static final double SHOOTER_KP = 0.0007;
    private static final double SHOOTER_KI = 0.0005;
    private static final double SHOOTER_KD = 0.0;
    private static final double TICKS_PER_REV = 28.0;
    private ElapsedTime timer = new ElapsedTime();
    private double currentVelocity, now, currentRPM, error, lastError, integralSum, derivative, output, deltaTime;
    private double lastTimestamp = 0.0;

    double kP = 0.0007;
    double kI = 0.0005;
    double kD = 0;
    double d = 0;

    private String ball1 = "null";
    private String ball2 = "null";
    private String ball3 = "null";
    private int id;

    @Override
    public void init(){
        lastTimestamp = timer.seconds();
        integralSum = 0;
        lastError = 0;

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gate = hardwareMap.get(CRServo.class, "gate");
        aprilTagWebcam.init(hardwareMap, telemetry);
    }
    public void updateShooterPID(double TARGET_RPM) {
        currentVelocity = shooterMotor.getVelocity();
        currentRPM = (currentVelocity / 28.0) * 60.0;

        now = timer.seconds();
        deltaTime = now - lastTimestamp;
        lastTimestamp = now;

        if (deltaTime <= 0) deltaTime = 0.001;

        error = TARGET_RPM - currentRPM;
        integralSum += error * deltaTime;
        derivative = (error - lastError) / deltaTime;

        output = (kP * error) + (kI * integralSum) + (kD * derivative);
        shooterMotor.setPower(output);

        lastError = error;
    }

    public void displayBallColorTelemetry(int detectedID){
        if (detectedID == 21) {
            ball1 = "Green";
            ball2 = "Purple";
            ball3 = "Purple";
        } else if (detectedID == 22) {
            ball1 = "Purple";
            ball2 = "Green";
            ball3 = "Purple";
        } else if (detectedID == 23) {
            ball1 = "Purple";
            ball2 = "Purple";
            ball3 = "Green";
        }

        telemetry.addData("ball1Color: ", ball1);
        telemetry.addData("ball2Color: ", ball2);
        telemetry.addData("ball3Color: ", ball3);
    };

    @Override
    public void loop() {
        aprilTagWebcam.update();

        AprilTagDetection det24 = aprilTagWebcam.getTagBySpecificID(24);
        aprilTagWebcam.displayDetectionTelemetry(det24);

        AprilTagDetection det20 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(det20);

        AprilTagDetection det21 = aprilTagWebcam.getTagBySpecificID(21);

        AprilTagDetection det22 = aprilTagWebcam.getTagBySpecificID(22);

        AprilTagDetection det23 = aprilTagWebcam.getTagBySpecificID(23);

        if (det21 != null) {
            id = 21;
        }

        if (det22 != null) {
            id = 22;
        }

        if (det23 != null) {
            id = 23;
        }

        displayBallColorTelemetry(id);

        if (det24 != null) {
            d = det24.ftcPose.y;
        }

        double dInches = d / 2.54;

        double rpm = aprilTagWebcam.getShooterRPM(dInches);
        updateShooterPID(rpm);

        if (Math.abs(currentRPM - rpm) < 150) {
            gate.setPower(-1);
        } else {
            gate.setPower(1);
        }

        telemetry.addData("Distance (in)", dInches);
        telemetry.addData("Output", output);
        telemetry.addData("Target RPM", rpm);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.update();
    }
}
