package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;
@Autonomous(name="BackMoveOutBlue", group = "OCE")
public class BackMoveOutBlue extends OpMode {
    private final Pose start = new Pose(0, 0, Math.toRadians(90));
    private final Pose end = new Pose(-18, 5, Math.toRadians(90));
    private Path path1;
    private int pathState = 0;
    private Follower follower;

    public void buildPaths() {
        path1 = new Path(new BezierLine(start, end));
        path1.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(path1);
            setPathState(1);
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);

        buildPaths();
    }

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
    }
}
