package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.List;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name = "A Centerstage Tele field Centric with AprilTagOmni")
public class CenterstageTele extends LinearOpMode {
    HardwarePowerpuffs robot = new HardwarePowerpuffs();
   // public String fieldOrRobotCentric="field";// pick up the centric
    public String fieldOrRobotCentric="robot";
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 4.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;        // Desired turning power/speed (-1 to +1)
    public float speedMultiplier = 0.5f;
    public float speedLimiter = 0.05f;
    boolean move = false;
    private static final int POSITION_Y = 5;
    private static final int POSITION_A = 0;
    private static final double SLIDE_POWER = 0.05; // Adjust as needed
    //apriltag related


    @Override public void runOpMode() {
            robot.init(hardwareMap);
            initAprilTag();
            if (USE_WEBCAM)
                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
            // Wait for driver to press start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();

            waitForStart();
            while (opModeIsActive()){

            targetFound = false;
            desiredTag = null;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }


            if (fieldOrRobotCentric.equals("field")) {
                FieldCentricDriveTrain();
                liftArmHigh();
                if (gamepad1.right_trigger > 0.3) { //close
                    robot.ClawR.setPosition(0.71);
                }
                if (gamepad1.left_trigger > 0.3) { //close
                    robot.ClawL.setPosition(0.505);
                }
                if (gamepad1.left_bumper && !move) { //open
                    robot.ClawL.setPosition(0.2);
                }
                if (gamepad1.right_bumper && !move) { //open
                    robot.ClawR.setPosition(0.5);
                }
                if (gamepad2.dpad_down && !move) { //down
                    robot.ArmR.setPosition(0);
                    robot.ArmL.setPosition(0);
                }
                if (gamepad2.dpad_up && !move) { //up
                    robot.ArmL.setPosition(0.95);
                    robot.ArmR.setPosition(0.95);
                }
                if (gamepad2.b && !move) { //up
                    robot.Wrist.setPosition(1);
                }
                if (gamepad2.x && !move) { //down
                    robot.Wrist.setPosition(0.6);

                }
                if (gamepad2.left_bumper && !move) { //shoot
                    robot.Drone.setPosition(1);
                }
                if (gamepad2.a && !move) { //all the way down
                    moveSlideToPosition(POSITION_A);
                }
                if (gamepad2.y && !move) { //up controlled
                    moveSlideToPosition(POSITION_Y);
                }
                if (gamepad1.left_bumper && targetFound) {
                    AprilTagOmniCameraRear();
                }
                moveRobot(drive, strafe, turn);
                sleep(10);
            } else if (fieldOrRobotCentric.equals("robot")) {
                RobotCentricDriveTrain();
                liftArmHigh();
                if (gamepad1.right_trigger > 0.3) { //close
                    robot.ClawR.setPosition(0.71);
                }
                if (gamepad1.left_trigger > 0.3) { //close
                    robot.ClawL.setPosition(0.505);
                }
                if (gamepad1.left_bumper && !move) { //open
                    robot.ClawL.setPosition(0.2);
                }
                if (gamepad1.right_bumper && !move) { //open
                    robot.ClawR.setPosition(0.5);
                }
                if (gamepad2.dpad_down && !move) { //down
                    robot.ArmR.setPosition(0);
                    robot.ArmL.setPosition(0);
                }
                if (gamepad2.dpad_up && !move) { //up
                    robot.ArmL.setPosition(0.95);
                    robot.ArmR.setPosition(0.95);
                }
                if (gamepad2.b && !move) { //up
                    robot.Wrist.setPosition(1);
                }
                if (gamepad2.x && !move) { //down
                    robot.Wrist.setPosition(0.6);

                }
                if (gamepad2.left_bumper && !move) { //shoot
                    robot.Drone.setPosition(1);
                }
                if (gamepad2.a && !move) { //all the way down
                    moveSlideToPosition(POSITION_A);
                }
                if (gamepad2.y && !move) { //up controlled
                    moveSlideToPosition(POSITION_Y);
                }
                if (gamepad1.left_bumper && targetFound) {
                    AprilTagOmniCameraRear();
                }
                moveRobot(drive, strafe, turn);
                sleep(10);
            }

//

        }
    }

    public void liftArmHigh() {
        double y = gamepad2.left_stick_y;
        robot.liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorL.setPower(y);
        robot.liftMotorR.setPower(y);

    }

    private void moveSlideToPosition(int targetPosition) {
        robot.liftMotorL.setTargetPosition(targetPosition);
        robot.liftMotorR.setTargetPosition(targetPosition);
        robot.liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorR.setPower(SLIDE_POWER);
        robot.liftMotorL.setPower(SLIDE_POWER);
        move=true;
        while (robot.liftMotorL.isBusy() && robot.liftMotorR.isBusy() && move) {
            // Wait until the motor reaches the target position
        }

        robot.liftMotorL.setPower(0);
        robot.liftMotorR.setPower(0);
        robot.liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        move=false;
    }
    public void FieldCentricDriveTrain() {
        //for gobilda motor with REV hub and Frist SDK, we need reverse all control signals
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        robot.LFMotor.setPower(0.75*frontLeftPower);
        robot.LBMotor.setPower(0.75*backLeftPower);
        robot.RFMotor.setPower(0.75*frontRightPower);
        robot.RBMotor.setPower(0.75*backRightPower);
    }

    public void RobotCentricDriveTrain(){
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double fl = y - x - rx;
        double bl = y + x - rx;
        double fr = y + x + rx;
        double br = y - x + rx;

        robot.LFMotor.setPower(fl*speedMultiplier);
        robot.LBMotor.setPower(bl*speedMultiplier);
        robot.RFMotor.setPower(fr*speedMultiplier);
        robot.RBMotor.setPower(br*speedMultiplier);

    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void AprilTagOmniCameraRear() {

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double  headingError    = desiredTag.ftcPose.bearing;
        double  yawError        = desiredTag.ftcPose.yaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
//testing rear camera

        double leftFrontPower    =  x -y +yaw;
        double rightFrontPower   =  x +y -yaw;
        double leftBackPower     =  x +y +yaw;
        double rightBackPower    =  x -y -yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        robot.LFMotor.setPower(leftFrontPower);
        robot.RFMotor.setPower(rightFrontPower);
        robot.LBMotor.setPower(leftBackPower);
        robot.RBMotor.setPower(rightBackPower);
    }

}