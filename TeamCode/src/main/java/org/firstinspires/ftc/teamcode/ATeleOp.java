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

@TeleOp(name = "ATeleOp [DRIFTFIX-v2 2026-10-08]")
public class ATeleOp extends LinearOpMode {

    // Bump this string every time you push a new build to the robot.
    // It shows up on the Driver Station so you can confirm the code that's
    // actually running is the one you just built (not a stale/cached APK).
    private static final String BUILD_TAG = "ATeleOp DRIFTFIX-v2 2026-10-08";

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

        // Diagnostic banner: if you don't see exactly this on the Driver
        // Station BEFORE pressing start, the robot is running an old/cached
        // build and you need to re-deploy, not change the code again.
        telemetry.addLine(BUILD_TAG);
        telemetry.addData("Status", "Hardware initialized, IMU ready - press START");
        telemetry.addData("Gamepad1", gamepad1.getGamepadId() != -1 ? "connected" : "NOT CONNECTED");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        imu.resetYaw();
        double targetHeading = 0;
        long loopCount = 0;

        while (opModeIsActive()) {
            loopCount++;

            // Driver can re-zero the heading hold at any time (e.g. after a
            // collision knocks the robot off its intended heading).
            if (gamepad1.back) {
                imu.resetYaw();
                targetHeading = 0;
            }

            double rawY = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double rawX = gamepad1.left_stick_x;
            double rawRx = gamepad1.right_stick_x;

            double y = rawY;
            double x = rawX * 1.1; // Counteract imperfect strafing
            double rx = rawRx;

            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = targetHeading - currentHeading;
            if (headingError > 180) headingError -= 360;
            else if (headingError < -180) headingError += 360;

            boolean correctionActive = Math.abs(rawRx) <= TURN_DEADZONE;

            if (!correctionActive) {
                // Driver is actively turning: just follow the stick, and keep
                // the heading target glued to wherever the turn leaves us.
                targetHeading = currentHeading;
            } else {
                // Driver wants to go straight (or stand still): steer back
                // towards the locked heading instead of letting it drift.
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

            // --- Diagnostics: everything you need to confirm this exact
            // build is running, and to tune the drift correction. ---
            telemetry.addLine(BUILD_TAG);
            telemetry.addData("Loop #", loopCount);
            telemetry.addLine();
            telemetry.addData("Stick raw", "y=%.2f x=%.2f rx=%.2f", rawY, rawX, rawRx);
            telemetry.addData("Drive cmd", "y=%.2f x=%.2f rx=%.2f", y, x, rx);
            telemetry.addData("Heading", "current=%.1f target=%.1f error=%.1f",
                    currentHeading, targetHeading, headingError);
            telemetry.addData("Drift correction", correctionActive ? "ACTIVE" : "off (driver turning)");
            telemetry.addLine();
            telemetry.addData("Motor FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.addData("Arm position", "%.2f", leftArm.getPosition());
            telemetry.update();
        }
    }
}
