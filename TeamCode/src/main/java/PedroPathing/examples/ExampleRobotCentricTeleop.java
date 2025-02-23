package PedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import PedroPathing.constants.FConstants;
import PedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@Disabled
@TeleOp(name = "Example Robot-Centric Teleop", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {
    private Follower follower;

    private final Pose collectPose = new Pose(6, 32, Math.toRadians(0));
    private final Pose scorePose = new Pose(32, 72, Math.toRadians(0));

    private final Path collectSpecimens = new Path(new BezierLine(new Point(scorePose), new Point(collectPose)));
    private final Path scoreSpecimens = new Path(new BezierLine(new Point(collectPose), new Point(scorePose)));

    private boolean autoMove = false;
    private boolean moveToCollect = false;
    private boolean lastYButtonState = false;

    /** This method is called once when init is pressed, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(follower.getPose().getHeading()))); // Start Position
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        boolean currentYButtonState = gamepad1.y;

        if (currentYButtonState && !lastYButtonState) {
            if (!autoMove) {
                moveToCollect = true;
                autoMove = true;
            } else {
                moveToCollect = !moveToCollect;
            }
        }
        lastYButtonState = currentYButtonState;

        Path targetPath = moveToCollect ? collectSpecimens : scoreSpecimens;

        if (autoMove && drive == 0 && strafe == 0 && turn == 0) {
            follower.followPath(targetPath, true);
            follower.update();
        } else {
            if (autoMove) {
                autoMove = false;
                moveToCollect = true;
            }
            follower.setTeleOpMovementVectors(drive, strafe, turn, true);
        }

        follower.update();

        /* Telemetry Outputs */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Auto Move Active", autoMove);
        telemetry.addData("Current Target", moveToCollect ? "Collecting Specimens" : "Scoring Specimens");
        telemetry.update();
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}