package org.firstinspires.ftc.teamcode.manual;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="motor_test", group = "Test")
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor w = hardwareMap.get(DcMotor.class, "w");

        telemetry.addData("Status", "Initialized. Press START to run motor 'w' at 1.0.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            w.setPower(-1.0);

            telemetry.addData("Status", "Motor 'w' at 1.0");
            telemetry.addData("Power", "%.2f", w.getPower());
            telemetry.update();
        }

        w.setPower(0);
    }
}

