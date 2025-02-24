package Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import PedroPathing.constants.FConstants;
import PedroPathing.constants.LConstants;

import Robot.Robot;

@Autonomous(name = "5-Specimen Auto", group = "Autonomous")
public class FiveSpecAuto extends OpMode {

    private final Robot robot = new Robot();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    /* ------- Define Robot Poses ------- */
    private final Pose startPose = new Pose(7, 72, Math.toRadians(0));  // Start Position
    private final Pose initalScorePose = new Pose(24, 78, Math.toRadians(0));
    private final Pose scorePose1 = new Pose(24, 75, Math.toRadians(0));
    private final Pose scorePose2 = new Pose(24, 72, Math.toRadians(0));
    private final Pose scorePose3 = new Pose(24, 69, Math.toRadians(0));
    private final Pose scorePose4 = new Pose(24, 66, Math.toRadians(0));
    private final Pose collectPose = new Pose(6, 32, Math.toRadians(0));
    private final Pose parkPose = new Pose(12, 24, Math.toRadians(0));  // Parking Position

    /* ------- Define Paths and PathChains ------- */
    private Path scorePreload, park;
    private PathChain collectSamples, scoreSpecimen1, scoreSpecimen2, scoreSpecimen3, scoreSpecimen4;

    public void buildPaths() {
        /* -------- Score Preload Path -------- */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(initalScorePose)));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));

        /* -------- Collect Samples PathChain -------- */
        collectSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(24.000, 72.000, Point.CARTESIAN), new Point(22.000, 48.000, Point.CARTESIAN), new Point(42.000, 40.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130))
                .addPath(new BezierLine(new Point(42.000, 40.000, Point.CARTESIAN), new Point(30.000, 40.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(60))
                .addPath(new BezierLine(new Point(30.000, 40.000, Point.CARTESIAN), new Point(42.000, 30.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(130))
                .addPath(new BezierLine(new Point(42.000, 30.000, Point.CARTESIAN), new Point(30.000, 30.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(60))
                .addPath(new BezierLine(new Point(30.000, 30.000, Point.CARTESIAN), new Point(42.000, 22.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(130))
                .addPath(new BezierLine(new Point(42.000, 22.000, Point.CARTESIAN), new Point(30.000, 22.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(60))
                .addPath(new BezierCurve(new Point(30.000, 22.000, Point.CARTESIAN), new Point(30.000, 32.000, Point.CARTESIAN), new Point(collectPose)))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();

        /* -------- Score Specimens Path -------- */
        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(collectPose), new Point(scorePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(scorePose1), new Point(collectPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(collectPose), new Point(scorePose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(scorePose2), new Point(collectPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(collectPose), new Point(scorePose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(scorePose3), new Point(collectPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(collectPose), new Point(scorePose4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(scorePose4), new Point(collectPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* -------- Park Path -------- */
        park = new Path(new BezierLine(new Point(collectPose), new Point(parkPose)));
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
                    follower.followPath(collectSamples);
                    setPathState(2);
                }
                break;

            case 2: // Score Specimens
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpecimen1);
                    setPathState(3);
                }
                break;

            case 3: // Score Specimens
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpecimen2);
                    setPathState(4);
                }
                break;

            case 4: // Score Specimens
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpecimen3);
                    setPathState(5);
                }
                break;

            case 5: // Score Specimens
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpecimen4);
                    setPathState(6);
                }
                break;

            case 6: // Park
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