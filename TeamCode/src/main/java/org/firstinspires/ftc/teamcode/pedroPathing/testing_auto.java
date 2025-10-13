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

@Autonomous(name = "ScorePreload", group = "Over-caffeinated")
public class testing_auto extends OpMode {
    private double shooterPower = -1;
    private double gatePower = -1;
    private Follower follower;
    private Timer pathTimer;
    private DcMotor shooterMotor = null;
    private CRServo gate = null;
    private int pathState;
    private double waitTime = 1000;
    private boolean waiting = false;
    private final Pose startPose = new Pose(0, 0);
    private final Pose scorePose = new Pose(-60, 0);
    private Path scorePreload;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                pathTimer.resetTimer();
                telemetry.addLine("Moving Back");
                break;
            case 1:
                shooterMotor.setPower(shooterPower);
                gate.setPower(gatePower);
//                if (pathTimer.getElapsedTimeSeconds() >= 1){
//                    telemetry.addLine("Shooting");
//
//                } else {
//                    telemetry.addLine("Waiting");
//                    setPathState(1);
//                }
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Shooter Power", shooterMotor.getPower());
        telemetry.update();


    }


    /** This method is called once at the init of the OpMode. **/
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