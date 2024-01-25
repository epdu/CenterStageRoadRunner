package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LinearSlide extends OpMode {
//    DcMotor liftMotorL;
    DcMotor liftMotorR;

    boolean move = false;

    private static final double SLIDE_POWER = 0.9; // Adjust as needed
    private static final int POSITION_A = 1000;   // Adjust these positions as needed
    private static final int POSITION_B = 2000;
    private static final int POSITION_X = 0;
    private static final int POSITION_Y = 2500;
    public float speedLimiter = 0.5f;


    @Override
    public void init() {
//        liftMotorL = hardwareMap.get(DcMotor.class, "liftMotorL");
        liftMotorR = hardwareMap.get(DcMotor.class, "liftMotorR");

//        int positionL = liftMotorL.getCurrentPosition();
        int positionR = liftMotorR.getCurrentPosition();

        liftMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
//        liftMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        if (gamepad1.a && !move) {
            moveSlideToPosition(POSITION_A);
        }  else if (gamepad1.b && !move) {
            moveSlideToPosition(POSITION_B);
        }   else if (gamepad1.y && !move) {
            moveSlideToPosition(POSITION_Y);
        }   else if (gamepad1.x && !move) {
            moveSlideToPosition(POSITION_X);
        }   else  {
            liftArmHigh();
        }

    }
    private void moveSlideToPosition(int targetPosition) {
//        liftMotorL.setTargetPosition(targetPosition);
        liftMotorR.setTargetPosition(targetPosition);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setPower(SLIDE_POWER);
//        liftMotorL.setPower(SLIDE_POWER);
        move=true;
        while (liftMotorR.isBusy() && move) {
//        while (liftMotorL.isBusy() && liftMotorR.isBusy() && move) {
            // Wait until the motor reaches the target position
        }

//        liftMotorL.setPower(0);
        liftMotorR.setPower(0);
//        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        move=false;
    }

    public void liftArmHigh(){
        double y = - gamepad1.left_stick_y;
//        liftMotorL.setPower(speedLimiter * y);
        liftMotorR.setPower(speedLimiter * y);

    }

}