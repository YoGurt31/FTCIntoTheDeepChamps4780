package Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import PedroPathing.constants.FConstants;
import PedroPathing.constants.LConstants;

import Robot.Robot;

@Disabled
@Autonomous(name = "TESTDS", group = "Autonomous")
public class PushAuto extends OpMode {

    private final Robot robot = new Robot();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /**
     * The current path state of the autonomous sequence
     */
    private int pathState;

    /* ------- Define Robot Poses ------- */
    private final Pose startPose = new Pose(7, 72, Math.toRadians(0));  // Start Position
    private final Pose initalScorePose = new Pose(32, 78, Math.toRadians(0));
    private final Pose collectPose1 = new Pose(60, 24, Math.toRadians(0));
    private final Pose collectPose2 = new Pose(60, 12, Math.toRadians(0));
    private final Pose collectPose3 = new Pose(60, 6, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(32, 75, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(32, 72, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(32, 69, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(32, 66, Math.toRadians(0));
    private final Pose claimPose = new Pose(12, 12, Math.toRadians(0));
    private final Pose parkPose = new Pose(12, 24, Math.toRadians(0));  // Parking Position

    /* ------- Define Paths and PathChains ------- */
    private Path scorePreload, park;
    private PathChain collectSample1, collectSample2, collectSample3, scoreSpecimen1, scoreSpecimen2, scoreSpecimen3, scoreSpecimen4;

    /**
     * Builds all paths before autonomous starts
     */
    public void buildPaths() {
        /* -------- Score Preload Path -------- */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(initalScorePose)));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));

        /* -------- Collect Samples PathChain -------- */
        collectSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initalScorePose), new Point(collectPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(collectPose1), new Point(claimPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        collectSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(claimPose), new Point(collectPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(collectPose2), new Point(claimPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        collectSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(claimPose), new Point(collectPose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(collectPose3), new Point(claimPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* -------- Score Specimens Path -------- */
        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(claimPose), new Point(scorePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(scorePose1), new Point(claimPose)))
                .build();

        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(claimPose), new Point(scorePose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(scorePose2), new Point(claimPose)))
                .build();

        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(claimPose), new Point(scorePose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(scorePose3), new Point(claimPose)))
                .build();

        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(claimPose), new Point(scorePose4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(scorePose4), new Point(claimPose)))
                .build();

        /* -------- Park Path -------- */
        park = new Path(new BezierLine(new Point(claimPose), new Point(parkPose)));
        park.setConstantHeadingInterpolation(Math.toRadians(0));
    }

    /**
     * Updates the autonomous sequence based on `pathState`
     */
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Score Preloaded Specimen
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // Collect Samples
                if (!follower.isBusy()) {
                    follower.followPath(collectSample1);
                    setPathState(2);
                }
                break;

            case 2: // Collect Samples
                if (!follower.isBusy()) {
                    follower.followPath(collectSample2);
                    setPathState(3);
                }
                break;

            case 3: // Collect Samples
                if (!follower.isBusy()) {
                    follower.followPath(collectSample3);
                    setPathState(4);
                }
                break;

            case 4: // Score Specimens
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpecimen1);
                    setPathState(5);
                }
                break;

            case 5: // Score Specimens
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpecimen2);
                    setPathState(6);
                }
                break;

            case 6: // Score Specimens
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpecimen3);
                    setPathState(7);
                }
                break;

            case 7: // Score Specimens
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpecimen4);
                    setPathState(8);
                }
                break;

            case 8: // Park
                if (!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * Updates the state machine
     */
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /**
     * Runs repeatedly during the OpMode
     */
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X Pos", follower.getPose().getX());
        telemetry.addData("Y Pos", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * Runs once when the OpMode initializes
     */
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /**
     * Runs continuously after `init()` but before `start()`
     */
    @Override
    public void init_loop() {
    }

    /**
     * Runs once when the OpMode starts
     */
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * Runs once when the OpMode stops
     */
    @Override
    public void stop() {
    }
}