package Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
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
@Autonomous(name = "Specimen", group = "Autonomous")
public class SpecimenAuto extends OpMode {

    private final Robot robot = new Robot();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private int specimenScored;

    private final double SPECIMEN_OFFSET = 2;

    private final Pose startPose = new Pose(7, 72, Math.toRadians(0));                                         // Start Position
    private final Pose collectPose = new Pose(6, 32, Math.toRadians(0));                                       // Collection Zone
    private final Pose parkPose = new Pose(12, 24, Math.toRadians(0));                                         // Parking Position

    private Pose getScorePose() {
        return new Pose(32 + (SPECIMEN_OFFSET * specimenScored), 72, Math.toRadians(0));
    }

    private Path scorePreload, scoreSpecimens, park;
    private PathChain collectSamples;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(getScorePose())));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));

        collectSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(32.000, 72.000, Point.CARTESIAN), new Point(24.000, 48.000, Point.CARTESIAN), new Point(42.000, 40.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140))
                .addPath(new BezierLine(new Point(42.000, 40.000, Point.CARTESIAN), new Point(30.000, 40.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(60))
                .addPath(new BezierLine(new Point(30.000, 40.000, Point.CARTESIAN), new Point(42.000, 30.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(140))
                .addPath(new BezierLine(new Point(42.000, 30.000, Point.CARTESIAN), new Point(30.000, 30.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(60))
                .addPath(new BezierLine(new Point(30.000, 30.000, Point.CARTESIAN), new Point(42.000, 22.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(140))
                .addPath(new BezierLine(new Point(42.000, 22.000, Point.CARTESIAN), new Point(30.000, 22.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(60))
                .addPath(new BezierCurve(new Point(30.000, 22.000, Point.CARTESIAN), new Point(30.000, 32.000, Point.CARTESIAN), new Point(collectPose)))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();

        scoreSpecimens = new Path(new BezierLine(new Point(collectPose), new Point(getScorePose())));
        scoreSpecimens.setConstantHeadingInterpolation(Math.toRadians(0));

        park = new Path(new BezierLine(new Point(follower.getPose()), new Point(parkPose)));
        park.setConstantHeadingInterpolation(Math.toRadians(0));
    }

    public void setPathState(int State) {
        pathState = State;
        pathTimer.resetTimer();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Score Preloaded Specimen
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    setPathState(1);
                }
                break;

            case 1: // Collect Samples
                if (!follower.isBusy()) {
                    follower.followPath(collectSamples);
                    setPathState(3);
                }
                break;

            case 2: // Score Specimen
                if (!follower.isBusy()) {
                    follower.followPath(scoreSpecimens);
                }

                if (specimenScored < 3) {
                    if (!follower.isBusy()) {
                        setPathState(1);
                        specimenScored++;
                    }
                } else {
                    setPathState(3);
                }
                break;

            case 3: // Park
                if (!follower.isBusy()) {
                    follower.followPath(park);
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Specimen Scored", specimenScored);
        telemetry.addData("X Pos", follower.getPose().getX());
        telemetry.addData("Y Pos", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

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

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}