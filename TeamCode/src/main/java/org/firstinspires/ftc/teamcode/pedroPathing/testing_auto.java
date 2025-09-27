package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "testing_auto_OCE", group = "Over-caffeinated")
public class testing_auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(0, 24, Math.toRadians(90));

    private final Pose secondPose = new Pose(24, 24, Math.toRadians(90));
    // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Path scorePreload;
    private PathChain secondRun;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());



        secondRun = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondPose.getHeading())
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
                follower.followPath(secondRun);
                setPathState(2);
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