package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.WIP.Constants;
@Disabled
public class BackMoveOutRed extends OpMode {
    private final Pose corner1 = new Pose(0, 0, 0);
    private final Pose corner2 = new Pose(-21.67, 21.69, 60);
    private final Pose corner3 = new Pose(21.67, -21.69, -60);
    private Path path1;
    private PathChain path2, path3;
    private int pathState = 0;
    private Follower follower;

    public void buildPaths() {
        path1 = new Path(new BezierLine(corner1, corner2));
        path1.setLinearHeadingInterpolation(corner1.getHeading(), corner2.getHeading());

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(corner2, corner3))
                .setLinearHeadingInterpolation(corner2.getHeading(), corner3.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(corner2, corner3))
                .setLinearHeadingInterpolation(corner3.getHeading(), corner1.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;
            case 1:
                follower.followPath(path2);
                setPathState(2);
                break;
            case 2:
                follower.followPath(path3);
                setPathState(3);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(corner1);

        buildPaths();
    }

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
    }

}
