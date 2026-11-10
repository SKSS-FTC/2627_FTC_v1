package org.firstinspires.ftc.teamcode.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "open day teleOP", group = "TeleOP")
public class openDayTeleOP extends LinearOpMode {
    private DcMotor leftUp, rightUp, leftDown, rightDown;
    private DcMotor shootl, shootr;
    private Servo claw;

    private final ElapsedTime runtime = new ElapsedTime();

    // Shooter toggle state
    private boolean shooterOn = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {

        leftUp    = hardwareMap.get(DcMotor.class, "leftUp");
        rightUp   = hardwareMap.get(DcMotor.class, "rightUp");
        leftDown  = hardwareMap.get(DcMotor.class, "leftDown");
        rightDown = hardwareMap.get(DcMotor.class, "rightDown");
        shootl    = hardwareMap.get(DcMotor.class, "shootl");
        shootr    = hardwareMap.get(DcMotor.class, "shootr");
        claw      = hardwareMap.get(Servo.class, "servo");

        // Reverse one side so positive power drives forward on both sides
        leftUp.setDirection(DcMotor.Direction.REVERSE);
        leftDown.setDirection(DcMotor.Direction.REVERSE);
        shootl.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        // Brake when power is zero
        for (DcMotor m : new DcMotor[]{leftUp, rightUp, leftDown, rightDown, shootl, shootr}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        claw.setPosition(0.95); // 0.95 for standby 0.75 for shoot

        telemetry.addLine("Ready. Left bumper = shooter toggle");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // --- Mecanum drive ---
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double leftFront  = y + x + rx;
            double rightFront = y - x - rx;
            double leftRear   = y - x + rx;
            double rightRear  = y + x - rx;

            double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                    Math.max(Math.abs(leftRear),  Math.abs(rightRear)));
            if (max > 1.0) {
                leftFront  /= max;
                rightFront /= max;
                leftRear   /= max;
                rightRear  /= max;
            }

            leftUp.setPower(leftFront);
            rightUp.setPower(rightFront);
            leftDown.setPower(leftRear);
            rightDown.setPower(rightRear);

            // --- Shooter toggle: press left bumper once to start, press again to stop ---
            boolean leftBumper = gamepad1.left_bumper;
            if (leftBumper && !lastLeftBumper) {   // rising edge only
                shooterOn = !shooterOn;
            }
            lastLeftBumper = leftBumper;

            double shootPower = shooterOn ? 0.5 : 0.0;
            shootl.setPower(shootPower);
            shootr.setPower(shootPower);

            // --- Claw: A = open, X = close (moved off bumper to avoid conflict) ---
            if (gamepad1.triangle) {
                claw.setPosition(0.75);
                sleep(200);
                claw.setPosition(0.95);
            }

            // --- Telemetry ---
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Shooter ON", shooterOn);
            telemetry.addData("LeftBumper", leftBumper);
            telemetry.addData("Claw", claw.getPosition());
            telemetry.update();
        }
    }
}
