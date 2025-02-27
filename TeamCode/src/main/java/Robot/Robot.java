// Coded By Gurtej Singh
// adb connect 192.168.43.1:5555

package Robot;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Robot {

    // DriveTrain Subsystem
    public class DriveTrain {
        public DcMotor frontLeft, frontRight, backLeft, backRight;
        public GoBildaPinpointDriver Odo;

        public void init(HardwareMap hwMap) {
            frontLeft = hwMap.dcMotor.get("frontLeft");     // Config 0
            frontRight = hwMap.dcMotor.get("frontRight");   // Config 1
            backLeft = hwMap.dcMotor.get("backLeft");       // Config 2
            backRight = hwMap.dcMotor.get("backRight");     // Config 3

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Odo = hwMap.get(GoBildaPinpointDriver.class,"PinPoint");

            Odo.setOffsets(140, 100); // Adjust these based on your configuration
            Odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            Odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            Odo.resetPosAndIMU();
        }

        public Pose2D getPosition() {
            Odo.update();
            return Odo.getPosition();
        }

        public double getHeadingInDegrees() {
            double headingInDegrees = Math.toDegrees(Odo.getHeading());
            return headingInDegrees;
        }

        public void updatePinPoint() {
            Odo.update();
        }

        public void updatePinPointHeading() {
            Odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        }

        public void resetPinPoint() {
            Odo.resetPosAndIMU();
        }

        public void mecDrive(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }

        public void runDriveTrainEncoders() {
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void runWithoutDriveTrainEncoders() {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void resetDriveTrainEncoders() {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void brake() {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }


    // Scoring Subsystem
    public class Scoring {
        public DcMotor horizontalSlideExtension, verticalSlide1, verticalSlide2, intakeRollers;
        public Servo intakePivot, clawPrimaryPivot, clawSecondaryPivot, clawStatus, sweeper;
        public RevBlinkinLedDriver LED;
        public ColorSensor colorSensor;

        public void init(HardwareMap hwMap) {
            horizontalSlideExtension = hwMap.dcMotor.get("horizontalSlide");  // Config 0
            horizontalSlideExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            horizontalSlideExtension.setDirection(DcMotorSimple.Direction.FORWARD);
            horizontalSlideExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalSlide1 = hwMap.dcMotor.get("verticalSlide1");    // Config 1
            verticalSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalSlide1.setDirection(DcMotorSimple.Direction.REVERSE);
            verticalSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            verticalSlide2 = hwMap.dcMotor.get("verticalSlide2");    // Config 2
            verticalSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
            verticalSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intakeRollers = hwMap.dcMotor.get("intakeRollers");      // Config 3
            intakeRollers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeRollers.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeRollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            sweeper = hwMap.servo.get("sweeper");

            intakePivot = hwMap.servo.get("intakePivot");

            clawPrimaryPivot = hwMap.servo.get("clawPrimaryPivot");

            clawSecondaryPivot = hwMap.servo.get("clawSecondaryPivot");

            clawStatus = hwMap.servo.get("clawStatus");

            LED = hwMap.get(RevBlinkinLedDriver.class, "LED");
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

            colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
            colorSensor.enableLed(true);
        }

        public void runScoringEncoders() {
            horizontalSlideExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            verticalSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeRollers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void resetScoringEncoders() {
            horizontalSlideExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    // Created Instances of Subsystems
    public DriveTrain driveTrain = new DriveTrain();
    public Scoring scoring = new Scoring();

    // Initialize Robot.Robot Hardware
    public void init(HardwareMap hwMap) {
        driveTrain.init(hwMap);
        scoring.init(hwMap);
    }
}