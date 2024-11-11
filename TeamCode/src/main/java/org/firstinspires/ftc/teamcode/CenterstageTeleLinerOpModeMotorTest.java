package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "CenterstageTeleLinerOpModeMotorTest ")
public class CenterstageTeleLinerOpModeMotorTest extends LinearOpMode {
    HardwarePowerpuffs robot = new HardwarePowerpuffs();
    public String fieldOrRobotCentric="robot";
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
    public float speedMultiplier = 0.9f;
    public float speedLimiter = 0.05f;
    boolean move = false;
    //    private static final int POSITION_Y = -600;
    private static final int POSITION_Y = 1600;//1322 //1000 one stage
    private static final int POSITION_A = 100;
    private static final int POSITION_PrepareForHing = 1600;
    private static final int POSITION_ForHing = 400;
    private static final double SLIDE_POWER = 0.8; // Adjust as needed

    //apriltag related
    @Override public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

                if (gamepad1.left_bumper && targetFound) {
                    double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
//                RobotCentricDriveTrain();
             RobotCentricMotorsTest();
//                liftArmHigh();

                    if (gamepad2.a && !move) { //all the way down
                        moveSlideToPosition(POSITION_A);
                    }
                    if (gamepad2.y && !move) { //up controlled
                        moveSlideToPosition(POSITION_Y);
                    }
                    if (gamepad2.dpad_left && !move) { //up controlled
                        moveSlideToPosition(POSITION_PrepareForHing);
                    }
                    if (gamepad2.dpad_right && !move) { //up controlled
                        moveSlideToPosition(POSITION_ForHing);
                    }

//                moveRobot(drive, strafe, turn);
//                    sleep(10);
//

            }
        }
    public void RobotCentricMotorsTest() {
        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double robot_x = gamepad1.left_stick_x;
        double robot_rx = gamepad1.right_stick_x;
//            IMU imu = hardwareMap.get(IMU.class, "imu");
//            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
//                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
//            imu.initialize(parameters);
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double fl = robot_y - robot_x - robot_rx;
        double bl = robot_y + robot_x - robot_rx;
        double fr = robot_y + robot_x + robot_rx;
        double br = robot_y - robot_x + robot_rx;

        robot.LFMotor.setPower(fl * speedMultiplier);
        robot.LBMotor.setPower(bl * speedMultiplier);
        robot.RFMotor.setPower(fr * speedMultiplier);
        robot.RBMotor.setPower(br * speedMultiplier);
        double lfMotorRPM = robot.LFMotor.getVersion() * 60 / 537.6; // Convert ticks per second to RPM
        double lbMotorRPM = robot.LBMotor.getVersion() * 60 / 537.6;
        double rfMotorRPM = robot.RFMotor.getVersion() * 60 / 537.6;
        double rbMotorRPM = robot.RBMotor.getVersion() * 60 / 537.6;

        telemetry.addData("LFMotor RPM", lfMotorRPM);
        telemetry.addData("LBMotor RPM", lbMotorRPM);
        telemetry.addData("RFMotor RPM", rfMotorRPM);
        telemetry.addData("RBMotor RPM", rbMotorRPM);

        telemetry.update();
    }



    public void moveRobot ( double moveRobot_x, double moveRobot_y, double moveRobot_yaw){

        double leftFrontPower = moveRobot_x - moveRobot_y + moveRobot_yaw;
        double rightFrontPower = moveRobot_x + moveRobot_y - moveRobot_yaw;
        double leftBackPower = moveRobot_x + moveRobot_y + moveRobot_yaw;
        double rightBackPower = moveRobot_x - moveRobot_y - moveRobot_yaw;

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
        sleep(10);
    }

    public void FieldCentricDriveTrain () {
        //for gobilda motor with REV hub and Frist SDK, we need reverse all control signals
        double field_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double field_x = -gamepad1.left_stick_x;
        double field_rx = -gamepad1.right_stick_x;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = field_x * Math.cos(-botHeading) - field_y * Math.sin(-botHeading);
        double rotY = field_x * Math.sin(-botHeading) + field_y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(field_rx), 1);
        double frontLeftPower = (rotY + rotX + field_rx) / denominator;
        double backLeftPower = (rotY - rotX + field_rx) / denominator;
        double frontRightPower = (rotY - rotX - field_rx) / denominator;
        double backRightPower = (rotY + rotX - field_rx) / denominator;

        robot.LFMotor.setPower(0.75 * frontLeftPower);
        robot.LBMotor.setPower(0.75 * backLeftPower);
        robot.RFMotor.setPower(0.75 * frontRightPower);
        robot.RBMotor.setPower(0.75 * backRightPower);
    }



    public void RobotCentricDriveTrain () {
        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double robot_x = gamepad1.left_stick_x;
        double robot_rx = gamepad1.right_stick_x;
//            IMU imu = hardwareMap.get(IMU.class, "imu");
//            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
//                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
//            imu.initialize(parameters);
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double fl = robot_y - robot_x - robot_rx;
        double bl = robot_y + robot_x - robot_rx;
        double fr = robot_y + robot_x + robot_rx;
        double br = robot_y - robot_x + robot_rx;

        robot.LFMotor.setPower(fl * speedMultiplier);
        robot.LBMotor.setPower(bl * speedMultiplier);
        robot.RFMotor.setPower(fr * speedMultiplier);
        robot.RBMotor.setPower(br * speedMultiplier);

    }

    public void liftArmHigh () {
        double liftArm_y = gamepad2.left_stick_y;
        robot.liftMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorL.setPower(liftArm_y);
        robot.liftMotorR.setPower(liftArm_y);
        //up joystick makes the slides rotate clockwise on the out right side
        //when looking at the robots right side from the outside wall the slide pulley spins clockwise/to the right when the joystick is pushed up

    }

    private void moveSlideToPosition ( int targetPosition){
//            robot.liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("liftMotorR.getCurrentPosition()",robot.liftMotorR.getCurrentPosition());
        telemetry.addData("liftMotorL.getCurrentPosition()",robot.liftMotorL.getCurrentPosition());
        telemetry.update();
        robot.liftMotorL.setTargetPosition(-targetPosition);
        robot.liftMotorR.setTargetPosition(-targetPosition);
        robot.liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorR.setPower(+SLIDE_POWER);
        robot.liftMotorL.setPower(+SLIDE_POWER);
        move = true;

        while (robot.liftMotorL.isBusy() && robot.liftMotorR.isBusy() && move) {
            // Wait until the motor reaches the target position
        }

        robot.liftMotorL.setPower(0);
        robot.liftMotorR.setPower(0);

        robot.liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        robot.liftMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        move = false;
    }
}

