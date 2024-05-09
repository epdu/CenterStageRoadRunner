package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FtcOrientationTele extends OpMode {

    DcMotor LFMotor;
    DcMotor LBMotor;
    DcMotor RFMotor;
    DcMotor RBMotor;

    Servo Claw;

    boolean move = false;

    @Override
    public void init() {
        LFMotor = hardwareMap.dcMotor.get("LFMotor"); //control hub port 1
        LBMotor = hardwareMap.dcMotor.get("LBMotor"); //control hub port 0
        RFMotor = hardwareMap.dcMotor.get("RFMotor"); // expansion hub port 1
        RBMotor = hardwareMap.dcMotor.get("RBMotor"); // expansion hub port 0

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Claw = hardwareMap.servo.get("Claw"); // expansion hub servo port 2
        Claw.setPosition(0.7);
    }

    @Override
    public void loop() {
        moveDriveTrain();
            if (gamepad1.right_trigger > 0.3) { //open
                Claw.setPosition(0.5);
            }if (gamepad1.left_trigger > 0.3) { //close
                Claw.setPosition(0.7);
        }
    }

    public void moveDriveTrain() {
            double y = gamepad1.left_stick_y;
            double x =- gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;


            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            LFMotor.setPower(fl*0.5);
            LBMotor.setPower(bl*0.5);
            RFMotor.setPower(fr*0.5);
            RBMotor.setPower(br*0.5);

        }
}
