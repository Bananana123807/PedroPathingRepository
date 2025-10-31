package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

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
import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;

@Autonomous(name = "BlueSideFarAuto", group = "Over-caffeinated")
public class BlueSideFarAuto extends OpMode {
    private double shooterPower = -0.55;
    private double gatePower = -1;
    private Follower follower;
    private Timer pathTimer;
    private DcMotorEx shooterMotor = null;
    private CRServo gate = null;
    private int pathState;
    private final int TICKS_PER_REV = 28;
    private double TARGET_RPM = 0;
    private double currentRPM = 0;// Initial target RPM for the shooter

    // PID coefficients for flywheel control
    private double kP = 50;  // Proportional gain
    private double kI = 0.5; // Integral gain
    private double kD = 0.1;  // Derivative gain
    private double previousError = 0; // Previous error for derivative
    private double integral = 0;     // Integral of error
    private int counter = -1;
    private double output = 0;
    private final Pose startPose = new Pose(0, 0);
    private final Pose scorePose = new Pose(15, 0);
    private final Pose moveOutPose = new Pose(40, 0);
    private Path scorePreload;
    private PathChain moveOut;
    private double waitTime = 2000;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(23));

        moveOut = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, moveOutPose))
                .setLinearHeadingInterpolation(Math.toRadians(23),  Math.toRadians(0))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                shooterMotor.setPower(output);
                follower.followPath(scorePreload);
                telemetry.addLine("Moving Back");
                waitTime = 2000;
                break;
            case 1:
                if (TARGET_RPM - 100 > currentRPM && currentRPM < TARGET_RPM + 100) {
//                    shooterMotor.setPower(output);
                    gate.setPower(-1);
                    waitTime = 1000;
                } else {
                    counter -= 1;}
                break;
            case 2:
                gate.setPower(1);
                waitTime = 2000;
                break;
            case 3:
                if (TARGET_RPM - 100 > currentRPM && currentRPM < TARGET_RPM + 100) {
                    //shooterMotor.setPower(output);
                    gate.setPower(gatePower);
                    waitTime = 1000;
                } else {
                    counter -= 1;}
                break;
            case 4:
                gate.setPower(1);
                waitTime = 2000;
                break;
            case 5:
                if (TARGET_RPM - 100 > currentRPM && currentRPM < TARGET_RPM + 100){
                    //shooterMotor.setPower(output);
                    gate.setPower(gatePower);
                    waitTime = 1000;
                } else {
                    counter -= 1;
                }
                break;
            case 6:
                gate.setPower(0);
                shooterMotor.setPower(0);
                break;
            case 7:
                shooterMotor.setPower(0);
                follower.followPath(moveOut);
                telemetry.addLine("Moving Out");
                break;
        }
    }

    @Override
    public void loop() {
        double TargetFor_TARGETRPM = 0;
        double targetTicksPerSec = (TARGET_RPM / 60.0) * TICKS_PER_REV;

        // Get current shooter motor velocity
        double currentTicksPerSec = shooterMotor.getVelocity();
        double currentRPM = (currentTicksPerSec / TICKS_PER_REV) * 60.0;
        double PointCRPM=4425;
        TargetFor_TARGETRPM = 3600;

        double error = targetTicksPerSec - currentTicksPerSec;  // Current error in velocity
        integral += error * 0.05;  // Integrate the error over time (adjust 0.05 for sampling rate)
        double derivative = (error - previousError) / 0.05; // Derivative of the error

        kP=0.005;
        kI=0.00008;
        kD=0;
        TARGET_RPM=PointCRPM;

        TARGET_RPM = Math.max(0, Math.min(6000, TARGET_RPM));  // Clamp target RPM between 0 and 6000
        double output = (kP * error + kI * integral + kD * derivative);
        output = Math.max(0, Math.min(1, output));  // Clamp target RPM between 0 and 6000

        follower.update();

        double elapsedTime = pathTimer.getElapsedTime();
        shooterMotor.setPower(output);

        if (elapsedTime >= waitTime) {
            counter += 1;
            setPathState(counter);
            autonomousPathUpdate();
            shooterMotor.setPower(output);
        }


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.addData("Shooter RPM", "%.1f", currentRPM);
        telemetry.addData("Target RPM", "%.1f", TARGET_RPM);
        telemetry.addData("Shooter TPS", "%.1f", currentTicksPerSec);
        telemetry.addData("Output (PID)", "%.1f", output);
        telemetry.addData("Gate Power", gate.getPower());
        telemetry.update();


    }


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init(){
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        gate = hardwareMap.get(CRServo.class, "gate");

        pathTimer = new Timer();
        pathTimer.resetTimer();

        setPathState(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        telemetry.update();
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}