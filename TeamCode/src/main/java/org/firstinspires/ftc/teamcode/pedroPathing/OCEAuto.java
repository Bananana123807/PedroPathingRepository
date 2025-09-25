package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "OCE_Auto", group = "Autonomous")
public class OCEAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    
    private final Pose startPose  = new Pose(56, 8, Math.toRadians(90));
    private PathChain line1, line2, line3, line4, line5, line6, line7;
public void buildPaths() {


        line1 = follower.pathBuilder()
            .addPath(
                new BezierLine(new Pose(56.000, 8.000), new Pose(116.165, 120.349))
            )
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))
            .build();

        line2 = follower.pathBuilder()
            .addPath(
                new BezierCurve(
                    new Pose(116.165, 120.349),
                    new Pose(80.286, 83.021),
                    new Pose(117.613, 83.665)
                )
            )
            .setTangentHeadingInterpolation()
            .build();

        line3 = follower.pathBuilder()
            .addPath(
                new BezierLine(new Pose(117.613, 83.665), new Pose(116.165, 120.509))
            )
            .setConstantHeadingInterpolation(Math.toRadians(40))
            .build();

        line4 = follower.pathBuilder()
            .addPath(
                new BezierCurve(
                    new Pose(116.165, 120.509),
                    new Pose(67.897, 58.726),
                    new Pose(117.131, 59.209)
                )
            )
            .setTangentHeadingInterpolation()
            .build();

        line5 = follower.pathBuilder()
            .addPath(
                new BezierLine(new Pose(117.131, 59.209), new Pose(116.165, 120.349))
            )
            .setConstantHeadingInterpolation(Math.toRadians(40))
            .build();

        line6 = follower.pathBuilder()
            .addPath(
                new BezierCurve(
                    new Pose(116.165, 120.349),
                    new Pose(73.046, 33.788),
                    new Pose(117.453, 35.397)
                )
            )
            .setTangentHeadingInterpolation()
            .build();


        line7 = follower.pathBuilder()
            .addPath(
                new BezierLine(new Pose(117.453, 35.397), new Pose(116.326, 120.349))
            )
            .setConstantHeadingInterpolation(Math.toRadians(40))
            .build();
        }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(line1, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(line2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(line3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(line4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(line5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(line6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(line7, true);
                    setPathState(7);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}

