package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class CenterstageTele extends OpMode {
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    DcMotor liftMotorL;
    DcMotor liftMotorR;
    Servo ClawR;
    Servo ClawL;
    Servo Wrist;
    Servo ArmR;
    Servo ArmL;
    Servo Drone;
    public float speedMultiplier = 0.5f;
    public float speedLimiter = 0.5f;

    boolean move = false;

    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotorL = hardwareMap.get(DcMotor.class, "liftMotorL");
        liftMotorR = hardwareMap.get(DcMotor.class, "liftMotorR");
        int positionL = liftMotorL.getCurrentPosition();
        int positionR = liftMotorR.getCurrentPosition();

        liftMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        liftMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Drone = hardwareMap.get(Servo.class, "Drone");
        Drone.setPosition(0);

        Wrist = hardwareMap.get(Servo.class, "wrist");
        Wrist.setPosition(0.34);

        ClawR = hardwareMap.get(Servo.class, "ClawR");
        ClawL = hardwareMap.get(Servo.class, "ClawL");
        ClawR.setPosition(0.78);
        ClawL.setPosition(0.018);

        ArmL = hardwareMap.get(Servo.class, "ArmL");
        ArmR = hardwareMap.get(Servo.class, "ArmR");

//        ArmL.setPosition(0.5);
//        ArmR.setPosition(0.5);
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

        LFMotor.setPower(0.75*frontLeftPower);
        LBMotor.setPower(0.75*backLeftPower);
        RFMotor.setPower(0.75*frontRightPower);
        RBMotor.setPower(0.75*backRightPower);
    }

    public void liftArmHigh() {
        double y = -gamepad1.left_stick_y;
        liftMotorL.setPower(speedLimiter * y);
        liftMotorR.setPower(speedLimiter * y);
    }

    @Override
    public void loop() {
        FieldCentricDriveTrain();
        liftArmHigh();
        if (gamepad1.right_trigger > 0.3) {
            ClawR.setPosition(0.78);
        }
        if (gamepad1.left_trigger > 0.3) {
            ClawL.setPosition(0.02);
        }
        if (gamepad1.left_bumper && !move) {
            ClawL.setPosition(0);
        }
        if (gamepad1.right_bumper && !move) {
            ClawR.setPosition(0.5);
        }
        if (gamepad1.a && !move) {
            ArmR.setPosition(0);
            ArmL.setPosition(0);
        }
        if (gamepad1.b && !move) {
            ArmL.setPosition(0.4);
            ArmR.setPosition(0.4);
        }
        if (gamepad2.b && !move) {
            Wrist.setPosition(0.545);
        }
        if (gamepad2.x && !move) {
            Wrist.setPosition(0.318);
        }
        if (gamepad2.y && !move) {
            Drone.setPosition(1);
        }
        if (gamepad1.left_trigger > 0.3) {
            ClawL.setPosition(0.5);
        }

    }
}



