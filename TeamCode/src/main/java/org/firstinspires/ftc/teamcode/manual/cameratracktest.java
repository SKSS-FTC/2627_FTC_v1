package org.firstinspires.ftc.teamcode.manual;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="gyrotrack_tap_snap", group = "Concept")
public class cameratracktest extends LinearOpMode {

    // ---- Tuning ----
    final double DESIRED_HEADING       = 178;    // Target angle
    final double TURN_GAIN             = 0.015;  // P gain
    final double TURN_DERIVATIVE_GAIN  = 0.008;  // D gain (damping)
    final double MAX_AUTO_TURN         = 0.15;   // Absolute max auto-turn power
    final double MIN_TURN_POWER        = 0.08;   // Breakaway power (must move robot)
    final double HEADING_THRESHOLD     = 3.0;    // Deadband in degrees
    final double SLOW_ZONE             = 20.0;   // Degrees within which we slow down
    final double SLOW_ZONE_MIN_FACTOR  = 0.6;    // Fraction of MAX_AUTO_TURN at 0° (0.6*0.15=0.09 > 0.08)
    final double SETTLE_RATE_DEG       = 0.6;    // deg/loop — must be below this to exit
    final double STICK_DEADZONE        = 0.05;

    private DcMotor leftUp = null;
    private DcMotor rightUp = null;
    private DcMotor leftDown = null;
    private DcMotor rightDown = null;
    private IMU imu = null;

    private boolean isAligning = false;
    private boolean lastBumperState = false;
    private double lastHeadingError = 0.0;

    @Override
    public void runOpMode() {
        leftUp    = hardwareMap.get(DcMotor.class, "leftUp");
        rightUp   = hardwareMap.get(DcMotor.class, "rightUp");
        leftDown  = hardwareMap.get(DcMotor.class, "leftDown");
        rightDown = hardwareMap.get(DcMotor.class, "rightDown");

        leftUp.setDirection(DcMotor.Direction.REVERSE);
        leftDown.setDirection(DcMotor.Direction.REVERSE);
        rightUp.setDirection(DcMotor.Direction.FORWARD);
        rightDown.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry.addData("Status", "Initialized. Tap Left Bumper to Snap.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---- Read sticks + deadzone ----
            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;
            if (Math.abs(drive)  < STICK_DEADZONE) drive  = 0;
            if (Math.abs(strafe) < STICK_DEADZONE) strafe = 0;
            if (Math.abs(turn)   < STICK_DEADZONE) turn   = 0;

            // ---- Bumper rising edge starts a snap ----
            if (gamepad1.left_bumper && !lastBumperState) {
                isAligning = true;
                lastHeadingError = 0.0;
            }
            lastBumperState = gamepad1.left_bumper;

            // ---- Driver override cancels snap ----
            if (Math.abs(turn) > 0.1 || Math.abs(drive) > 0.1 || Math.abs(strafe) > 0.1) {
                isAligning = false;
            }

            // ---- Read heading + wrap error ----
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentHeading = orientation.getYaw(AngleUnit.DEGREES);

            double headingError = DESIRED_HEADING - currentHeading;
            headingError = Math.toDegrees(Math.atan2(
                    Math.sin(Math.toRadians(headingError)),
                    Math.cos(Math.toRadians(headingError))));

            // ---- Alignment control ----
            if (isAligning) {
                double headingRate = Math.abs(headingError - lastHeadingError); // deg/loop

                // Exit only when close AND slow
                if (Math.abs(headingError) <= HEADING_THRESHOLD
                        && headingRate < SETTLE_RATE_DEG) {
                    isAligning = false;
                    lastHeadingError = 0.0;
                    turn = 0.0;
                } else {
                    // Slow zone: shrink the cap near the target, never below breakaway
                    double factor;
                    if (Math.abs(headingError) >= SLOW_ZONE) {
                        factor = 1.0;
                    } else {
                        factor = SLOW_ZONE_MIN_FACTOR
                                + (1.0 - SLOW_ZONE_MIN_FACTOR) * (Math.abs(headingError) / SLOW_ZONE);
                    }
                    double maxTurn = MAX_AUTO_TURN * factor;
                    // Hard guarantee: cap can't drop below breakaway
                    if (maxTurn < MIN_TURN_POWER) maxTurn = MIN_TURN_POWER;

                    // PD
                    double pTerm = headingError * TURN_GAIN;
                    double dTerm = (lastHeadingError - headingError) * TURN_DERIVATIVE_GAIN;
                    turn = pTerm + dTerm;

                    // Friction comp (before clip, so the clip cannot undo it)
                    if (Math.abs(turn) < MIN_TURN_POWER) {
                        turn = Math.signum(headingError) * MIN_TURN_POWER;
                    }

                    // Cap
                    turn = Range.clip(turn, -maxTurn, maxTurn);

                    lastHeadingError = headingError;
                }
            } else {
                lastHeadingError = 0.0;
            }

            // ---- Mecanum kinematics ----
            double leftUpPower    = drive + strafe + turn;
            double rightUpPower   = drive - strafe - turn;
            double leftDownPower  = drive - strafe + turn;
            double rightDownPower = drive + strafe - turn;

            double max = Math.max(Math.abs(leftUpPower),  Math.abs(rightUpPower));
            max = Math.max(max, Math.abs(leftDownPower));
            max = Math.max(max, Math.abs(rightDownPower));
            if (max > 1.0) {
                leftUpPower    /= max;
                rightUpPower   /= max;
                leftDownPower  /= max;
                rightDownPower /= max;
            }

            leftUp.setPower(leftUpPower);
            rightUp.setPower(rightUpPower);
            leftDown.setPower(leftDownPower);
            rightDown.setPower(rightDownPower);

            // ---- Telemetry ----
            telemetry.addData("Mode", isAligning
                    ? String.format("ALIGNING TO %.0f°", DESIRED_HEADING) : "MANUAL");
            telemetry.addData("Current Heading", "%6.1f°", currentHeading);
            telemetry.addData("Heading Error",   "%6.1f°", headingError);
            telemetry.addData("Heading Rate",    "%6.2f°/loop",
                    Math.abs(headingError - lastHeadingError));
            telemetry.addData("Applied Turn",    "%6.2f", turn);
            telemetry.update();
        }
    }
}