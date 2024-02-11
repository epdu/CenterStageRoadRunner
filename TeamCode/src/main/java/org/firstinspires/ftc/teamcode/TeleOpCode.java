package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOpCode extends OpMode {
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    DcMotor liftMotorL;
    DcMotor liftMotorR;
    public Servo LauncherServo;

    boolean move = false;

    //    private static final double SLIDE_POWER = 0.9; // Adjust as needed
//    private static final int POSITION_A = 500;   // Adjust these positions as needed
//    private static final int POSITION_Y = 0;
    public float speedMultiplier = 0.5f;
    public float speedLimiter = 0.5f;
    private Servo servo;
    public boolean previous1 = false;
    public boolean previous2 = false;

    private DcMotorEx motor;
    DcMotor Intake;
    public Servo IntakeServo;
    public float speedMultiplier2 = 0.8f;
    public float speedMultiplier1 = 1;

    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        liftMotorL = hardwareMap.get(DcMotor.class, "liftMotorL");
//        liftMotorR = hardwareMap.get(DcMotor.class, "liftMotorR");

        liftMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//*********without servo on robot please Comment these lines for debuging

//        LauncherServo = hardwareMap.get(Servo.class, "LauncherServo");
//
//        servo = hardwareMap.get(Servo.class, "Servo");
//        motor = hardwareMap.get(DcMotorEx.class, "Motor");
//        servo.setPosition(0.5);
//
//        Intake = hardwareMap.get(DcMotor.class, "Intake");
//        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
//        LauncherServo.setPosition(0);
//        IntakeServo.setPosition(0);
//**************
    }

    @Override
    public void loop() {
//        moveDriveTrain();
        FieldCentricDriveTrain();
//        if (gamepad2.y && !move) {
//            moveSlideToPosition(POSITION_Y);
//        }  else if (gamepad2.a && !move) {
//            moveSlideToPosition(POSITION_A);}
//         if (gamepad1.right_trigger > 0.3) {
//             motor.setPower(0.5);
//         }
        if (gamepad2.x && !move) {
            LauncherServo.setPosition(0.25);
        }
        if (gamepad2.a && !move) {
            IntakeServo.setPosition(0.5);
        }
        if (gamepad2.b && !move) {
            IntakeServo.setPosition(0);
        } else {
//            liftArmHigh();
            moveServo(gamepad1.dpad_up, gamepad1.dpad_down);
            moveIntake(gamepad2.dpad_up, gamepad2.dpad_down);

            motor.setPower(gamepad2.left_stick_y * 0.5);

            telemetry.addData("SERVO", servo.getPosition());
            telemetry.update();

            ejectPixel();
        }

    }

    //*********Beginning of Field Centric
    public void FieldCentricDriveTrain() {
        //for gobilda motor with REV hub and Frist SDK, we need reverse all control signals
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
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

        LFMotor.setPower(frontLeftPower);
        LBMotor.setPower(backLeftPower);
        RFMotor.setPower(frontRightPower);
        RBMotor.setPower(backRightPower);
    }

//*********End of Field Centric


//    private void moveSlideToPosition(int targetPosition) {
//        liftMotorL.setTargetPosition(targetPosition);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotorL.setPower(SLIDE_POWER);
//        move=true;
//        while (liftMotorL.isBusy() && move) {
//            // Wait until the motor reaches the target position
//        }
//        liftMotorL.setPower(0);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        move=false;
//    }

//    public void liftArmHigh(){
//        double y = - gamepad1.left_stick_y;
//        liftMotorL.setPower(speedLimiter * y);

    //    }
    public void moveServo(boolean keybind1, boolean keybind2) {
        boolean current1 = keybind1;
        if (current1 && !previous1) {
            servo.setPosition(servo.getPosition() + 0.1);
        }
        previous1 = current1;

        boolean current2 = keybind2;
        if (current2 && !previous2) {
            servo.setPosition(servo.getPosition() - 0.1);
        }
        previous2 = current2;
    }

    public void moveIntake(boolean dpad_up, boolean dpad_down) {
        double intake = gamepad2.right_trigger;
        Intake.setPower(-intake * speedMultiplier1);
    }

    public void ejectPixel() {
        double intake = gamepad2.left_trigger;
        Intake.setPower(intake * speedMultiplier2);
    }

    public void liftArmHigh() {
        double y = -gamepad1.left_stick_y;
        liftMotorL.setPower(speedLimiter * y);
        liftMotorR.setPower(speedLimiter * y);
//*********Robot-Centric
//
//    public void moveDriveTrain() {
//        double y = gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;
//
//        double fl = y - x - rx;
//        double bl = y + x - rx;
//        double fr = y + x + rx;
//        double br = y - x + rx;
//
//        LFMotor.setPower(fl*speedMultiplier);
//        LBMotor.setPower(bl*speedMultiplier);
//        RFMotor.setPower(fr*speedMultiplier);
//        RBMotor.setPower(br*speedMultiplier);
//
//        telemetry.addData("y",y);
//        telemetry.addData("x",x);
//        telemetry.addData("fl",fl);
//        telemetry.addData("bl",bl);
//        telemetry.addData("fr",fr);
//        telemetry.addData("br",br);
//        telemetry.addData("fl*speedMultiplier",fl*speedMultiplier);
//        telemetry.addData("bl*speedMultiplier",bl*speedMultiplier);
//        telemetry.addData("fr*speedMultiplier",fr*speedMultiplier);
//        telemetry.addData("br*speedMultiplier",br*speedMultiplier);
//
//        telemetry.update();
//
//
//    }
//*********Robot-Centric

    }
}

