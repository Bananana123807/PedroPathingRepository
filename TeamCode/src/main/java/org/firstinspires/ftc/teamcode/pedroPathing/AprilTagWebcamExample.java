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
    private static final double MAX_INTEGRAL = 500.0;
    private ElapsedTime timer = new ElapsedTime();
    private double currentVelocity, currentRPM, error, lastError, integralSum, derivative, output, deltaTime;
    private double lastTimestamp = 0.0;
    double i = 0;

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
        currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;

        error = TARGET_RPM - currentRPM;

        double currentTime = timer.seconds();
        deltaTime = currentTime - lastTimestamp;
        lastTimestamp = currentTime;

        if (deltaTime <= 0) return;

        integralSum += error * deltaTime;
        integralSum = Math.max(Math.min(integralSum, MAX_INTEGRAL), -MAX_INTEGRAL);

        derivative = (error - lastError) / deltaTime;

        output = (SHOOTER_KP * error) + (SHOOTER_KI * integralSum) + (SHOOTER_KD * derivative);

        output = Math.max(Math.min(output, 1.0), -1.0);

        shooterMotor.setPower(output);

        lastError = error;
    }

    @Override
    public void loop(){
        aprilTagWebcam.update();

        AprilTagDetection det = aprilTagWebcam.getTagBySpecificID(24);
        aprilTagWebcam.displayDetectionTelemetry(det);

        if (det != null) {
            double d = Math.sqrt(
                    det.ftcPose.x * det.ftcPose.x +
                            det.ftcPose.y * det.ftcPose.y +
                            det.ftcPose.z * det.ftcPose.z
            );

            double dInches = d / 2.54;

            double rpm = aprilTagWebcam.getShooterRPM(dInches);
            updateShooterPID(rpm);

            if (Math.abs(error) < 150) {
                gate.setPower(-1);
            } else {
                gate .setPower(0);
            }

            telemetry.addData("Distance (in)", dInches);
            telemetry.addData("Output", output);
            telemetry.addData("Target RPM", rpm);
        }

        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);

        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.addData("Shooter RPM", currentRPM);
    }
}
