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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import PedroPathing.constants.FConstants;
import PedroPathing.constants.LConstants;

import Robot.Robot;

@Autonomous(name = "Specimen", group = "Autonomous")
public class SpecimenAuto extends OpMode {

    private final Robot robot = new Robot();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0;
    private int specimenScored = 0;

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

//        collectSamples = new PathBuilder()
//                .addPath(new BezierCurve(new Point(32.000, 72.000, Point.CARTESIAN), new Point(24.000, 48.000, Point.CARTESIAN), new Point(48.000, 36.000, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
//                .addPath(new BezierLine(new Point(48.000, 36.000, Point.CARTESIAN), new Point(30.000, 36.000, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(60))
//                .addPath(new BezierLine(new Point(30.000, 36.000, Point.CARTESIAN), new Point(42.000, 27.000, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(120))
//                .addPath(new BezierLine(new Point(42.000, 27.000, Point.CARTESIAN), new Point(30.000, 27.000, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(60))
//                .addPath(new BezierLine(new Point(30.000, 27.000, Point.CARTESIAN), new Point(42.000, 18.000, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(120))
//                .addPath(new BezierLine(new Point(42.000, 18.000, Point.CARTESIAN), new Point(30.000, 18.000, Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(60))
//                .addPath(new BezierCurve(new Point(30.000, 18.000, Point.CARTESIAN), new Point(30.000, 32.000, Point.CARTESIAN), new Point(30.000, 32.000, Point.CARTESIAN), new Point(30.000, 32.000, Point.CARTESIAN), new Point(collectPose)))
//                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
//                .setReversed(true)
//                .build();

        collectSamples = new PathChain(
                new Path(new BezierCurve(new Point(32.000, 72.000, Point.CARTESIAN), new Point(24.000, 24.000, Point.CARTESIAN), new Point(84.000, 24.000, Point.CARTESIAN), new Point(60.000, 24.000, Point.CARTESIAN))),
                new Path(new BezierLine(new Point(60.000, 24.000, Point.CARTESIAN), new Point(20.000, 24.000, Point.CARTESIAN))),
                new Path(new BezierCurve(new Point(20.000, 24.000, Point.CARTESIAN), new Point(60.000, 36.000, Point.CARTESIAN), new Point(60.000, 12.000, Point.CARTESIAN))),
                new Path(new BezierLine(new Point(60.000, 12.000, Point.CARTESIAN), new Point(20.000, 12.000, Point.CARTESIAN))),
                new Path(new BezierCurve(new Point(20.000, 12.000, Point.CARTESIAN), new Point(60.158, 24.000, Point.CARTESIAN), new Point(60.000, 6.000, Point.CARTESIAN))),
                new Path(new BezierLine(new Point(60.000, 6.000, Point.CARTESIAN), new Point(20.000, 6.000, Point.CARTESIAN))),
                new Path(new BezierCurve(new Point(20.000, 6.000, Point.CARTESIAN), new Point(36.000, 36.000, Point.CARTESIAN), new Point(6.000, 32.000, Point.CARTESIAN)))
        );

        scoreSpecimens = new Path(new BezierLine(new Point(collectPose), new Point(getScorePose())));
        scoreSpecimens.setConstantHeadingInterpolation(Math.toRadians(0));

        park = new Path(new BezierLine(new Point(getScorePose()), new Point(parkPose)));
        park.setConstantHeadingInterpolation(Math.toRadians(0));
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        robot.scoring.intakePivot.setPosition(0.0);
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Specimen Scored", specimenScored);
        telemetry.addData("Current Position", follower.getPose().toString());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Score Preloaded Specimen
                buildPaths();
                follower.followPath(scorePreload, true);
                robot.scoring.sweeper.setPosition(0.0);
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 1: // Collect Samples
                buildPaths();
                follower.followPath(collectSamples, true);
                robot.scoring.sweeper.setPosition(0.75);
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 2: // Score Specimen
                buildPaths();
                follower.followPath(scoreSpecimens, true);
                robot.scoring.sweeper.setPosition(0.0);

                specimenScored++;
                if (specimenScored < 3) {
                    if (!follower.isBusy()) {
                        pathState = 1;
                    }
                } else {
                    if (!follower.isBusy()) {
                        pathState = 3;
                    }
                }
                break;

            case 3: // Park
                buildPaths();
                follower.followPath(park, true);
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }
    }
}