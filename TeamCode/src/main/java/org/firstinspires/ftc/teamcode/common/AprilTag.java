package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import android.util.Size;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag or AprilTag Cluster
 * A "Cluster" is a group of Apriltags that share a common origin, and are identified by name.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag/Cluster in the TagLibrary is detected, the SDK provides location and orientation of the target, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * For a single tag, the "Drive Target" is the center of the Tag.  For a cluster of tags, the "Drive Target" will be the
 * (0,0,0) ORIGIN of the cluster, which may have been positioned somewhere other than the center of the cluster in order
 * to help to locate a game objective.
 *
 * The driving goal is to rotate to keep the Target centered in the camera, while strafing to be directly in front of the target,
 *  and driving towards the target to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: front_left_drive and front_right_drive, back_left_drive and back_right_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID.
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Target centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Target, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Target to get to the desired distance.  (Use TargetRange to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp(name = "AprilTag", group = "Common")
public class AprilTag extends LinearOpMode {
    // Adjust these numbers to suit your robot.

    final double DESIRED_DISTANCE = 30.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;    //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;    //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;    //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftUp = null;
    private DcMotor rightUp = null;
    private DcMotor leftDown = null;
    private DcMotor rightDown = null;
    private final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private final int DESIRED_TAG_ID = -1;    // The tag you want to approach, or set to -1 for ANY tag.
    private final String DESIRED_CLUSTER_NAME = null; // The cluster name you want to approach, or set null for ANY cluster.
    private VisionPortal.Builder myVisionPortalBuilder;
    private VisionPortal myVisionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    private boolean targetFound = false;    // Set to true when an AprilTag/Cluster target is detected
    private String targetName = "none";
    private int targetID = 0;
    private double targetRange = 0;
    private double targetBearing = 0;
    private double targetYaw = 0;

    @Override
    public void runOpMode() {
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftUp = hardwareMap.get(DcMotor.class, "leftUp");
        rightUp = hardwareMap.get(DcMotor.class, "rightUp");
        leftDown = hardwareMap.get(DcMotor.class, "leftDown");
        rightDown = hardwareMap.get(DcMotor.class, "rightDown");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftUp.setDirection(DcMotor.Direction.REVERSE);
        leftDown.setDirection(DcMotor.Direction.REVERSE);
        rightUp.setDirection(DcMotor.Direction.FORWARD);
        rightDown.setDirection(DcMotor.Direction.FORWARD);

        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortalBuilder.addProcessor(aprilTag);
        myVisionPortalBuilder.setCameraResolution(new Size(720, 480));
        myVisionPortalBuilder.enableLiveView(true);
        myVisionPortal = myVisionPortalBuilder.build();

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            targetFound = false;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                telemetry.addLine(String.format("ID", singleDet.id, singleDet.metadata.name));
                telemetry.addData("Range", "%5.2f inch", detection.ftcPose.range);
                telemetry.addData("Bearing", "%5.2f inch", detection.ftcPose.bearing);
                telemetry.addData("X", "%5.2f inch", detection.ftcPose.x);
                telemetry.addData("Y", "%5.2f inch", detection.ftcPose.y);
                telemetry.addData("Yaw", "%5.2f inch", detection.ftcPose.yaw);


            }
        }
    }

}
