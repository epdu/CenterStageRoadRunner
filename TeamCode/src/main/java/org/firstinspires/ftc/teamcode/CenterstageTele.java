package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "A Centerstage Tele either field or robot")
public class CenterstageTele extends OpMode {
    HardwarePowerpuffs robot = new HardwarePowerpuffs();
   // public String fieldOrRobotCentric="field";// pick up the centri
    public String fieldOrRobotCentric="robot";
    public float speedMultiplier = 0.5f;
    public float speedLimiter = 0.05f;
    boolean move = false;
    private static final int POSITION_Y = 5;
    private static final int POSITION_A = 0;
    private static final double SLIDE_POWER = 0.05; // Adjust as needed

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(fieldOrRobotCentric.equals("field")) {
            FieldCentricDriveTrain();
        }else if (fieldOrRobotCentric.equals("robot")){
            RobotCentricDriveTrain();
        }

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
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
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
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        if (gamepad1.options) {
            imu.resetYaw();
        }
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double fl = y - x - rx;
        double bl = y + x - rx;
        double fr = y + x + rx;
        double br = y - x + rx;

        robot.LFMotor.setPower(fl*speedMultiplier);
        robot.LBMotor.setPower(bl*speedMultiplier);
        robot.RFMotor.setPower(fr*speedMultiplier);
        robot.RBMotor.setPower(br*speedMultiplier);

    }

}