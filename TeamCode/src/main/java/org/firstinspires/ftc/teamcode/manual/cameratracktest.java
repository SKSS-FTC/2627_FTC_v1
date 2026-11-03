package org.firstinspires.ftc.teamcode.manual;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.VariableLogger;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name="zn_tuned_gyro_snap", group = "Concept")
public class cameratracktest extends LinearOpMode {

    final boolean ZN_MODE = true;

    private double znGain = 0.005;

    // For the first sanity test after the sign fix, temporarily set this to 30.0
    // and start the robot near heading = 0°. Once that converges cleanly,
    // set it back to 178 for match use.
    final double DESIRED_HEADING       = 178;

    // Placeholder values — replaced after the post-fix Z-N sweep
    final double TURN_GAIN             = 0.012;
    final double TURN_DERIVATIVE_GAIN  = 0.003;
    final double TURN_INTEGRAL_GAIN    = 0.0;
    final double INTEGRAL_LIMIT        = 3.0;

    final double STICK_DEADZONE        = 0.05;
    final double OVERRIDE_THRESHOLD    = 0.10;
    final double SETTLE_GRACE_SEC      = 0.15;

    // ---- Logging config ----
    final double LOG_INTERVAL_SEC      = 0.020;  // 50 Hz sample rate
    final double DPAD_DEBOUNCE_SEC     = 0.15;

    private DcMotor leftUp = null, rightUp = null, leftDown = null, rightDown = null;
    private IMU imu = null;

    private boolean isAligning      = false;
    private boolean lastBumperState = false;
    private double  lastLoopTime    = 0.0;
    private double  integralError   = 0.0;
    private double  snapStartTime   = 0.0;

    // ---- Logging state ----
    private VariableLogger logger;
    private double lastLogTime  = 0.0;
    private double lastDpadTime = 0.0;

    // Reused each iteration — no per-loop allocation
    private final String[] LOG_NAMES  =
            {"aligning", "heading", "error", "turn", "yawRate", "znGain", "dtMs"};
    private final Object[] LOG_VALUES = new Object[LOG_NAMES.length];

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

        for (DcMotor m : new DcMotor[]{leftUp, rightUp, leftDown, rightDown}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        // One file per run — solves the append-header problem and makes trials easy to compare
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        logger = new VariableLogger("zn_" + stamp + ".csv");
        logger.log("run", "start;ZN=" + ZN_MODE + ";desired=" + DESIRED_HEADING
                + ";znGain=" + znGain);

        telemetry.addData("Status", "Initialized.");
        if (ZN_MODE) telemetry.addLine("ZN MODE: bumper toggles, D-pad up/down changes Ku");
        telemetry.update();

        waitForStart();
        lastLoopTime = getRuntime();
        lastLogTime  = lastLoopTime;

        try {
            while (opModeIsActive()) {
                double now = getRuntime();
                double dt  = Math.max(now - lastLoopTime, 1e-3);
                lastLoopTime = now;

                // ---- Sticks ----
                double drive  = -gamepad1.left_stick_y;
                double strafe =  gamepad1.left_stick_x;
                double turn   =  gamepad1.right_stick_x;
                if (Math.abs(drive)  < STICK_DEADZONE) drive  = 0;
                if (Math.abs(strafe) < STICK_DEADZONE) strafe = 0;
                if (Math.abs(turn)   < STICK_DEADZONE) turn   = 0;

                // ---- Bumper ----
                if (gamepad1.left_bumper && !lastBumperState) {
                    if (ZN_MODE) {
                        isAligning = !isAligning;
                    } else {
                        isAligning    = true;
                        snapStartTime = now;
                    }
                    integralError = 0.0;
                    logger.log("event", "bumper;aligning=" + isAligning + ";znGain=" + znGain);
                }
                lastBumperState = gamepad1.left_bumper;

                // ---- D-pad adjusts Ku (time-based debounce, no blocking sleep) ----
                if (ZN_MODE) {
                    if (gamepad1.dpad_up && now - lastDpadTime > DPAD_DEBOUNCE_SEC) {
                        znGain += 0.001;
                        lastDpadTime = now;
                        logger.log("event", "znGain=" + znGain);
                    }
                    if (gamepad1.dpad_down && now - lastDpadTime > DPAD_DEBOUNCE_SEC) {
                        znGain = Math.max(0.0, znGain - 0.001);
                        lastDpadTime = now;
                        logger.log("event", "znGain=" + znGain);
                    }
                }

                // ---- Driver override (full PID mode only) ----
                if (!ZN_MODE) {
                    boolean driverOverride =
                            Math.abs(turn)   > OVERRIDE_THRESHOLD ||
                                    Math.abs(drive)  > OVERRIDE_THRESHOLD ||
                                    Math.abs(strafe) > OVERRIDE_THRESHOLD;
                    if (driverOverride && (now - snapStartTime) > SETTLE_GRACE_SEC) {
                        isAligning = false;
                    }
                }

                // ---- Heading + yaw rate (read once, used everywhere) ----
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                double currentHeading = orientation.getYaw(AngleUnit.DEGREES);
                AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                double headingError = DESIRED_HEADING - currentHeading;
                headingError = Math.toDegrees(Math.atan2(
                        Math.sin(Math.toRadians(headingError)),
                        Math.cos(Math.toRadians(headingError))));

                // ---- Control ----
                if (isAligning) {
                    if (ZN_MODE) {
                        turn = headingError * znGain;
                    } else {
                        double pTerm = headingError * TURN_GAIN;
                        double dTerm = -av.zRotationRate * TURN_DERIVATIVE_GAIN;
                        integralError += headingError * dt;
                        integralError = Range.clip(integralError, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
                        double iTerm = integralError * TURN_INTEGRAL_GAIN;
                        turn = pTerm + dTerm + iTerm;
                    }
                    turn = -turn;   // sign fix: align controller output is opposite to stick
                } else {
                    integralError = 0.0;
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

                // ---- Log at 50 Hz (one flush per sample, not one per variable) ----
                if (now - lastLogTime >= LOG_INTERVAL_SEC) {
                    LOG_VALUES[0] = isAligning;
                    LOG_VALUES[1] = currentHeading;
                    LOG_VALUES[2] = headingError;
                    LOG_VALUES[3] = turn;
                    LOG_VALUES[4] = av.zRotationRate;
                    LOG_VALUES[5] = znGain;
                    LOG_VALUES[6] = dt * 1000.0;
                    logger.log(LOG_NAMES, LOG_VALUES);
                    lastLogTime = now;
                }

                // ---- Telemetry ----
                telemetry.addData("Mode", ZN_MODE
                        ? (isAligning ? "ZN P-TEST" : "ZN IDLE")
                        : (isAligning ? "PID ALIGN" : "MANUAL"));
                if (ZN_MODE) telemetry.addData("znGain (Ku candidate)", "%.4f", znGain);
                telemetry.addData("Heading",     "%6.1f°", currentHeading);
                telemetry.addData("Error",       "%6.1f°", headingError);
                telemetry.addData("Yaw Rate",    "%6.2f°/s", av.zRotationRate);
                telemetry.addData("Applied Turn","%6.3f", turn);
                telemetry.addData("dt",          "%5.1f ms", dt * 1000.0);
                telemetry.update();
            }
        } finally {
            logger.log("run", "stop");
            logger.close();
        }
    }
}