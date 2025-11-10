package org.firstinspires.ftc.teamcode.pedroPathing.WIP; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Disabled
@Autonomous(name = "testing_auto_OCE", group = "Over-caffeinated")
public class OCEAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(120.358, 120.944, Math.toRadians(45)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(88.706, 87.924, Math.toRadians(45));
    // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.706, 87.924),
                                new Pose(74.638, 74.247),
                                new Pose(120.749, 84.407)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.749, 84.407),
                                new Pose(74.638, 74.052),
                                new Pose(88.706, 87.924)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.706, 87.924),
                                new Pose(74.247, 50.801),
                                new Pose(120.944, 59.788)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.944, 59.788),
                                new Pose(74.247, 50.801),
                                new Pose(88.706, 87.924)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.706, 87.924),
                                new Pose(73.661, 26.182),
                                new Pose(120.749, 35.365)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(120.749, 35.365),
                                new Pose(73.465, 26.182),
                                new Pose(88.706, 87.924)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                follower.followPath(grabPickup1);
                setPathState(2);
                break;
            case 2:
                follower.followPath(scorePickup1);
                setPathState(3);
                break;
            case 3:
                follower.followPath(grabPickup2);
                setPathState(4);
                break;
            case 4:
                follower.followPath(scorePickup2);
                setPathState(5);
                break;
            case 5:
                follower.followPath(grabPickup3);
                setPathState(6);
                break;
            case 6:
                follower.followPath(scorePickup3);
                setPathState(7);
                break;
        }
    }
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}