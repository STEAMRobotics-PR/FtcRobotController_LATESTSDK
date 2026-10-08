package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ATeleOp [v3 2026-10-08]")
public class ATeleOp extends LinearOpMode {

    // Bump this string every time you push a new build to the robot.
    // It shows up on the Driver Station so you can confirm the code that's
    // actually running is the one you just built (not a stale/cached APK).
    private static final String BUILD_TAG = "ATeleOp v3 2026-10-08";

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
        telemetry.addData("Status", "Hardware initialized - press START");
        telemetry.addData("Gamepad1", gamepad1.getGamepadId() != -1 ? "connected" : "NOT CONNECTED");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        long loopCount = 0;

        while (opModeIsActive()) {
            loopCount++;

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

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

            // --- Diagnostics: confirms this exact build is the one running. ---
            telemetry.addLine(BUILD_TAG);
            telemetry.addData("Loop #", loopCount);
            telemetry.addLine();
            telemetry.addData("Stick", "y=%.2f x=%.2f rx=%.2f", y, x, rx);
            telemetry.addData("Motor FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.addData("Arm position", "%.2f", leftArm.getPosition());
            telemetry.update();
        }
    }
}
