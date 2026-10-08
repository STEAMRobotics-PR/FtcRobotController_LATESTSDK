package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class ATeleOp extends LinearOpMode {

    // How aggressively the IMU corrects back to the target heading when the
    // driver isn't actively turning. Raise it if the robot still drifts,
    // lower it if it starts to wobble/oscillate while driving straight.
    private static final double HEADING_CORRECTION_GAIN = 0.015;
    // Cap on how much turning power the correction is allowed to add.
    private static final double HEADING_CORRECTION_MAX = 0.3;
    // Right-stick inputs smaller than this are ignored and treated as
    // "the driver wants to go straight", so the correction can kick in.
    private static final double TURN_DEADZONE = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        //For motors add a new motor decleration
        //DcMotor intake = hardwareMap.dcMotor.get("intake");

        //For servos add a new servo decleration
        Servo leftArm = hardwareMap.servo.get("leftArm");

        // Built-in IMU on the Control/Expansion Hub. We use it to hold a
        // straight heading and counteract the drive pulling to one side
        // when the driver is just pushing the stick forward.
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // EDIT these two directions to match how YOUR hub is actually mounted
        // on the robot (see the SensorIMUOrthogonal sample for details).
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //For motors add a new brake mode
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        imu.resetYaw();
        double targetHeading = 0;

        while (opModeIsActive()) {
            // Driver can re-zero the heading hold at any time (e.g. after a
            // collision knocks the robot off its intended heading).
            if (gamepad1.back) {
                imu.resetYaw();
                targetHeading = 0;
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (Math.abs(rx) > TURN_DEADZONE) {
                // Driver is actively turning: just follow the stick, and keep
                // the heading target glued to wherever the turn leaves us.
                targetHeading = currentHeading;
            } else {
                // Driver wants to go straight (or stand still): steer back
                // towards the locked heading instead of letting it drift.
                double headingError = targetHeading - currentHeading;
                if (headingError > 180) headingError -= 360;
                else if (headingError < -180) headingError += 360;
                rx = Range.clip(headingError * HEADING_CORRECTION_GAIN,
                        -HEADING_CORRECTION_MAX, HEADING_CORRECTION_MAX);
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            //For motors add action commands

            //if (gamepad1.left_bumper) {
                //intake.setPower(1);
            //
            // }

            if (gamepad1.a) {
                leftArm.setPosition(0);
            }

            if (gamepad1.b) {
                leftArm.setPosition(1);
            }

            if (gamepad1.x) {
                leftArm.setPosition(0.5);
            }

            telemetry.addData("heading", "current %.1f / target %.1f", currentHeading, targetHeading);
            telemetry.update();
        }
    }
}
