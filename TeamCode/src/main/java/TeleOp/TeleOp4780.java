// Coded By Gurtej Singh

package TeleOp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import PedroPathing.constants.FConstants;
import PedroPathing.constants.LConstants;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import Robot.Robot;

@TeleOp(name = "TeleOpBlue", group = "TeleOp")
public class TeleOp4780 extends LinearOpMode {

    private final Robot robot = new Robot();
    private Follower follower;

    private final Pose collectPose = new Pose(6, 32, Math.toRadians(0));
    private final Pose scorePose = new Pose(32, 72, Math.toRadians(0));

    private final Path collectSpecimens = new Path(new BezierLine(new Point(scorePose), new Point(collectPose)));
    private final Path scoreSpecimens = new Path(new BezierLine(new Point(collectPose), new Point(scorePose)));

    private final double intakeLiftedPosition = 0.00;
    private final double intakeLoweredPosition = 0.40;

    IntakeState intakeState = IntakeState.IDLE;
    OuttakeState outtakeState = OuttakeState.COLLECTION;

    private boolean hasMoved = false;
    private boolean clawActionCompleted = false;
    private VerticalSlideAction verticalSlideAction = null;

    private boolean lastAButtonState = false;
    private boolean lastXButtonState = false;

    enum IntakeState {
        IDLE,
        ACTIVE
    }

    enum OuttakeState {
        COLLECTION,
        SCORING
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initialized! Activate Controller(s)...");

        robot.init(hardwareMap);
        robot.driveTrain.brake();
        robot.driveTrain.resetDriveTrainEncoders();
        robot.scoring.resetScoringEncoders();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(follower.getPose().getHeading()))); // Start Position

        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {

            robot.scoring.sweeper.setPosition(0.0); // (0.85) = Down

            robot.driveTrain.runDriveTrainEncoders();
            robot.scoring.runScoringEncoders();

            // Drive System
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            follower.setTeleOpMovementVectors(drive, strafe, turn, true);


            if (strafe == 0 && drive == 0) {
                robot.driveTrain.brake();
            }

            follower.update();

            // Drive Telemetry
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Target Pose", follower.getPose());
            telemetry.addLine();
            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Strafe", "%.2f", strafe);

            telemetry.addLine("\n");


//            // Horizontal Slide Extension Control
//            double horizontalSlideExtensionPower = 0;
//            if (gamepad1.dpad_down) {
//                horizontalSlideExtensionPower = -1.0;
//            }
//            if (gamepad1.dpad_up) {
//                horizontalSlideExtensionPower = 1.0;
//            }
//            int currentHorizontalSlidePosition = robot.scoring.horizontalSlideExtension.getCurrentPosition();
//            int horizontalSlideExtensionMax = 2500;  // Max Value
//            int horizontalSlideExtensionMin = -50;  // Min Value
//            int horizontalToleranceThreshold = 25;  // Threshold Value (Adjust)
//
//            if (currentHorizontalSlidePosition < horizontalSlideExtensionMax - horizontalToleranceThreshold) {
//                double adjustedPower = horizontalSlideExtensionPower * (1 - (double) (currentHorizontalSlidePosition - (horizontalSlideExtensionMax - horizontalToleranceThreshold)) / horizontalToleranceThreshold);
//                robot.scoring.horizontalSlideExtension.setPower(Math.min(horizontalSlideExtensionPower, adjustedPower));
//            } else if (robot.scoring.horizontalSlideExtension.getCurrentPosition() > horizontalSlideExtensionMin) {
//                double adjustedPower = horizontalSlideExtensionPower * (1 - (double) ((horizontalSlideExtensionMin + horizontalToleranceThreshold) - currentHorizontalSlidePosition) / horizontalToleranceThreshold);
//                robot.scoring.horizontalSlideExtension.setPower(Math.max(horizontalSlideExtensionPower, adjustedPower));
//            } else {
//                robot.scoring.horizontalSlideExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                robot.scoring.horizontalSlideExtension.setPower(0);
//            }
//
//            telemetry.addLine("----- Horizontal Slide Extension -----");
//            telemetry.addData("Power", "%.2f", horizontalSlideExtensionPower);
//            telemetry.addData("Position", "%d", robot.scoring.horizontalSlideExtension.getCurrentPosition());
//
//            telemetry.addLine("\n");
//
//
            // Intake Roller Control
            int Red = robot.scoring.colorSensor.red();
            int Green = robot.scoring.colorSensor.green();
            int Blue = robot.scoring.colorSensor.blue();

            boolean isYellow = Red > 200 && Green > 200 && Blue > 200;
            boolean isRed = Red > 200 && Green < 400 && Blue < 200;
            boolean isBlue = Red < 200 && Green < 400 && Blue > 200;
            boolean isIgnored = (Red > 45 && Red < 65) && (Green > 100 && Green < 120) && (Blue > 100 && Blue < 120);

            boolean manualForward = gamepad1.right_bumper;
            boolean manualReverse = gamepad1.left_bumper;

            if (manualForward) {
                robot.scoring.intakePivot.setPosition(intakeLoweredPosition);
            } else {
                robot.scoring.intakePivot.setPosition(intakeLiftedPosition);
            }

            if (isYellow) {
                robot.scoring.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (isRed) {
                robot.scoring.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (isBlue) {
                robot.scoring.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else {
                robot.scoring.LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            // Auto Color Control (BLUE)
            if (!manualForward && !manualReverse) {
                switch (intakeState) {
                    case IDLE:
                        if (isRed) {
                            intakeState = IntakeState.ACTIVE;
                        }

                        robot.scoring.intakeRollers.setPower(0);

                        break;

                    case ACTIVE:
                        robot.scoring.intakeRollers.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.scoring.intakeRollers.setPower(1);

                        if (isYellow || isBlue || isIgnored) {
                            intakeState = IntakeState.IDLE;
                            robot.scoring.intakeRollers.setPower(0);
                        }

                        break;
                }
            } else {
                if (manualForward) {
                    robot.scoring.intakeRollers.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.scoring.intakeRollers.setPower(1);
                } else if (manualReverse) {
                    robot.scoring.intakeRollers.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.scoring.intakeRollers.setPower(1);
                } else {
                    robot.scoring.intakeRollers.setPower(0);
                }
            }

            String rollerStatus = "Stopped";
            if (robot.scoring.intakeRollers.getPower() > 0) {
                if (robot.scoring.intakeRollers.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    rollerStatus = "Intaking";
                } else if (robot.scoring.intakeRollers.getDirection() == DcMotorSimple.Direction.REVERSE) {
                    rollerStatus = "Outtaking";
                }
            } else {
                rollerStatus = "Stopped";
            }

            telemetry.addLine("----- Roller Info -----");
            telemetry.addData("Intake Roller Status", rollerStatus);
            telemetry.addData("Servo Position", robot.scoring.intakePivot.getPosition());
            telemetry.addData("Current State", intakeState);

            telemetry.addLine("\n");

            telemetry.addLine("----- Color Sensor Info -----");
            telemetry.addData("Red: ", Red);
            telemetry.addData("Green: ", Green);
            telemetry.addData("Blue: ", Blue);
            telemetry.addLine();
            telemetry.addData("Is Yellow: ", isYellow);
            telemetry.addData("Is Blue: ", isBlue);
            telemetry.addData("Is Red: ", isRed);
            telemetry.addData("Is Ignored: ", isIgnored);

            telemetry.addLine("\n");


            // Vertical Slide Extension Control
            double verticalSlideExtensionPower = gamepad1.left_trigger - gamepad1.right_trigger;
            int currentVerticalSlidePosition = (robot.scoring.verticalSlide1.getCurrentPosition() + robot.scoring.verticalSlide2.getCurrentPosition()) / 2;
            int verticalSlideExtensionMin = -75;  // Min Value
            int verticalToleranceThreshold = 25;  // Threshold Value (Adjust)
            double slowFactor = 0.75;

            final double HOLD = 0.0005;
            boolean Holding = false;

            if (currentVerticalSlidePosition > 150 && robot.scoring.verticalSlide1.getCurrentPosition() < 900 && robot.scoring.verticalSlide2.getCurrentPosition() < 900 && verticalSlideExtensionPower == 0) {
                robot.scoring.verticalSlide1.setPower(HOLD);
                robot.scoring.verticalSlide2.setPower(HOLD);
                Holding = true;
            } else if (verticalSlideExtensionPower != 0 && currentVerticalSlidePosition > verticalSlideExtensionMin) {
                if (currentVerticalSlidePosition <= verticalToleranceThreshold) {
                    double adjustedPower = verticalSlideExtensionPower * slowFactor;
                    robot.scoring.verticalSlide1.setPower(adjustedPower);
                    robot.scoring.verticalSlide2.setPower(adjustedPower);
                } else {
                    robot.scoring.verticalSlide1.setPower(verticalSlideExtensionPower);
                    robot.scoring.verticalSlide2.setPower(verticalSlideExtensionPower);
                }
            } else {
                robot.scoring.verticalSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.scoring.verticalSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.scoring.verticalSlide1.setPower(0);
                robot.scoring.verticalSlide2.setPower(0);
            }

            telemetry.addLine("----- Diagonal Slide Extension -----");
            telemetry.addData("Power", "%.2f", verticalSlideExtensionPower);
            telemetry.addData("HOLD Power", Holding ? String.format("%.4f", HOLD) : "Not Holding");
            telemetry.addData("Position", "%d", currentVerticalSlidePosition);

            telemetry.addLine("\n");


            // Claw Control
            boolean currentXButtonState = gamepad1.x;
            if (currentXButtonState && !lastXButtonState) {
                if (robot.scoring.clawStatus.getPosition() == 0.60) {
                    robot.scoring.clawStatus.setPosition(0.425); // Open
                } else {
                    robot.scoring.clawStatus.setPosition(0.60); // Close
                }
            }
            lastXButtonState = currentXButtonState;


            // Outtake System Control
            boolean currentAButtonState = gamepad1.a;
            if (currentAButtonState && !lastAButtonState) {
                switch (outtakeState) {
                    case COLLECTION:
                        outtakeState = OuttakeState.SCORING;
                        hasMoved = false;
                        clawActionCompleted = false;
                        break;

                    case SCORING:
                        outtakeState = OuttakeState.COLLECTION;
                        hasMoved = false;
                        clawActionCompleted = false;
                        break;
                }
            }
            lastAButtonState = currentAButtonState;

            switch (outtakeState) {
                case COLLECTION: // Default
                    runClawActions(0.00, 0.075, 0.425);

                    if (clawActionCompleted && !hasMoved) {
                        verticalSlideAction = new VerticalSlideAction(15);
                        hasMoved = true;
                    }

                    if (verticalSlideAction != null && !verticalSlideAction.run(new TelemetryPacket())) {

                    } else {
                        verticalSlideAction = null;
                    }

                    break;

                case SCORING:
                    runClawActions(1.00, 1.00, 0.60);

                    if (clawActionCompleted && !hasMoved) {
                        verticalSlideAction = new VerticalSlideAction(850);
                        hasMoved = true;
                    }

                    if (verticalSlideAction != null && !verticalSlideAction.run(new TelemetryPacket())) {

                    } else {
                        verticalSlideAction = null;
                    }

                    break;
            }

            telemetry.addLine("----- Outtake System Status -----");
            telemetry.addData("Current Case", outtakeState);
            telemetry.addData("Primary Pivot Position", "%.2f", robot.scoring.clawPrimaryPivot.getPosition());
            telemetry.addData("Secondary Pivot Position", "%.2f", robot.scoring.clawSecondaryPivot.getPosition());

            double clawPosition = robot.scoring.clawStatus.getPosition();
            String clawStatus = clawPosition <= 0.425 ? "Open" : clawPosition > 0.425 ? "Closed" : "Partially Open";
            telemetry.addData("Claw Status", clawStatus);
            telemetry.addData("Claw Position", "%.2f", clawPosition);

            telemetry.addLine("\n");

            telemetry.update();
        }
    }

    private void runClawActions(double primaryPivot, double secondaryPivot, double clawPosition) {
        if (!clawActionCompleted) {
            robot.scoring.clawPrimaryPivot.setPosition(primaryPivot);
            robot.scoring.clawSecondaryPivot.setPosition(secondaryPivot);
            robot.scoring.clawStatus.setPosition(clawPosition);

            sleep(500);
            clawActionCompleted = true;
        }
    }

    public class VerticalSlideAction implements Action {
        private final int targetPosition;
        private boolean isStarted = false;

        // Decent Values
        private final double kP = 0.0200000000;
        private final double kI = 0.0000000100;
        private final double kD = 0.0000325000;

        private final int tolerance = 25;
        private final double HOLD = 0;

        private double integralSum = 0;
        private int lastError = 0;
        private long lastTime = System.nanoTime();

        public VerticalSlideAction(int position) {
            this.targetPosition = position;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isStarted) {
                isStarted = true;
                robot.scoring.verticalSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.scoring.verticalSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lastTime = System.nanoTime();
            }

            int currentPosition = (robot.scoring.verticalSlide1.getCurrentPosition() + robot.scoring.verticalSlide2.getCurrentPosition()) / 2;
            int error = targetPosition - currentPosition;

            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1e9;
            lastTime = currentTime;

            integralSum += error * deltaTime;

            double errorDerivative = (error - lastError) / deltaTime;

            double power = (kP * error) + (kI * integralSum) + (kD * errorDerivative);
            power = Math.max(-1, Math.min(1, power));

            robot.scoring.verticalSlide1.setPower(power);
            robot.scoring.verticalSlide2.setPower(power);

            lastError = error;

            if (Math.abs(error) < tolerance) {
                robot.scoring.verticalSlide1.setPower(HOLD);
                robot.scoring.verticalSlide2.setPower(HOLD);
                robot.scoring.verticalSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.scoring.verticalSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                return true;
            }

            return false;
        }
    }
}