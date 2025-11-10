package org.firstinspires.ftc.teamcode.pedroPathing.WIP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "RedSideAutoClose", group = "Over-caffeinated")
public class RedSideAutoClose extends OpMode {
    private double shooterPower = -0.7;
    private double gatePower = -1;
    private Follower follower;
    private Timer pathTimer;
    private DcMotor shooterMotor = null;
    private CRServo gate = null;
    private int pathState;
    private int counter = -1;
    private final Pose startPose = new Pose(0, 0);
    private final Pose scorePose = new Pose(-30, 0);
    private final Pose moveOutPose = new Pose(-30, -15);
    private Path scorePreload;
    private PathChain moveOut;
    private double waitTime = 2000;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));

        moveOut = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, moveOutPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                shooterMotor.setPower(shooterPower);
                follower.followPath(scorePreload);
                telemetry.addLine("Moving Back");
                waitTime = 2000;
                break;
            case 1:
                gate.setPower(gatePower);
                waitTime = 1500;
                break;
            case 2:
                gate.setPower(1);
                waitTime = 2000;
                break;
            case 3:
                gate.setPower(gatePower);
                waitTime = 750;
                break;
            case 4:
                gate.setPower(1);
                waitTime = 2000;
                break;
            case 5:
                gate.setPower(gatePower);
                waitTime = 1000;
                break;
            case 6:
                gate.setPower(0);
                break;
            case 7:
                follower.followPath(moveOut);
                telemetry.addLine("Moving Out");
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();

        double elapsedTime = pathTimer.getElapsedTime();

        if (elapsedTime >= waitTime) {
            counter += 1;
            setPathState(counter);
            autonomousPathUpdate();
        }


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.update();


    }



    @Override
    public void init(){

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        gate = hardwareMap.get(CRServo.class, "gate");

        pathTimer = new Timer();
        pathTimer.resetTimer();

        setPathState(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();


        telemetry.update();
    }
}
