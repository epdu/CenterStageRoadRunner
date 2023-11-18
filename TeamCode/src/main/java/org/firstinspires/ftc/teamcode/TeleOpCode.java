package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpCode extends OpMode {
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    DcMotor liftMotorL;
    public Servo LauncherServo;

    boolean move = false;

    private static final double SLIDE_POWER = 0.9; // Adjust as needed
    private static final int POSITION_A = 500;   // Adjust these positions as needed
    private static final int POSITION_Y = 0;
    public float speedMultiplier = 0.5f;
    public float speedLimiter = 0.5f;
    private Servo servo;
    public boolean previous1 = false;
    public boolean previous2 = false;

    private DcMotorEx motor;
    DcMotor Intake;
    public Servo IntakeServo;
    public float speedMultiplier2 = 1;
    public float speedMultiplier1 = 0.2f;
    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotorL = hardwareMap.get(DcMotor.class, "liftMotorL");

        int positionL = liftMotorL.getCurrentPosition();

        liftMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LauncherServo = hardwareMap.get(Servo.class, "LauncherServo");

        servo = hardwareMap.get(Servo.class, "Servo");
        motor = hardwareMap.get(DcMotorEx.class, "Motor");
        servo.setPosition(0.5);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");

    }
    @Override
    public void loop(){
        moveDriveTrain();
        if (gamepad2.y && !move) {
            moveSlideToPosition(POSITION_Y);
        }  else if (gamepad2.a && !move) {
            moveSlideToPosition(POSITION_A);
        } else if (gamepad2.x) {
            LauncherServo.setPosition(0.25);
        } else if (gamepad2.b && !move) {
            IntakeServo.setPosition(0.5);
        }else if (gamepad2.x && !move){
            IntakeServo.setPosition(0);
        }else {
            liftArmHigh();
            moveServo(gamepad1.dpad_up, gamepad1.dpad_down);
            motor.setPower(gamepad1.left_stick_y * 0.5);

            telemetry.addData("SERVO", servo.getPosition());
            telemetry.update();

            ejectPixel();}

    }


    public void moveDriveTrain() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        LFMotor.setPower(fl*speedMultiplier);
        LBMotor.setPower(bl*speedMultiplier);
        RFMotor.setPower(fr*speedMultiplier);
        RBMotor.setPower(br*speedMultiplier);

        telemetry.addData("y",y);
        telemetry.addData("x",x);
        telemetry.addData("fl",fl);
        telemetry.addData("bl",bl);
        telemetry.addData("fr",fr);
        telemetry.addData("br",br);
        telemetry.addData("fl*speedMultiplier",fl*speedMultiplier);
        telemetry.addData("bl*speedMultiplier",bl*speedMultiplier);
        telemetry.addData("fr*speedMultiplier",fr*speedMultiplier);
        telemetry.addData("br*speedMultiplier",br*speedMultiplier);

        telemetry.update();


    }
    private void moveSlideToPosition(int targetPosition) {
        liftMotorL.setTargetPosition(targetPosition);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorL.setPower(SLIDE_POWER);
        move=true;
        while (liftMotorL.isBusy() && move) {
            // Wait until the motor reaches the target position
        }
        liftMotorL.setPower(0);
        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        move=false;
    }

    public void liftArmHigh(){
        double y = - gamepad1.left_stick_y;
        liftMotorL.setPower(speedLimiter * y);

    }
    public void moveServo(boolean keybind1, boolean keybind2)
    {
        boolean current1 = keybind1;
        if(current1 && !previous1)
        {
            servo.setPosition(servo.getPosition() + 0.1);
        }
        previous1 = current1;

        boolean current2 = keybind2;
        if(current2 && !previous2)
        {
            servo.setPosition(servo.getPosition() - 0.1);
        }
        previous2=current2;
    }

    public void moveIntake(){
        double intake = gamepad2.right_trigger;
        Intake.setPower(intake*speedMultiplier);
    }

    public void ejectPixel(){
        double intake = gamepad2.left_trigger;
        Intake.setPower(intake*speedMultiplier1);
    }

}

