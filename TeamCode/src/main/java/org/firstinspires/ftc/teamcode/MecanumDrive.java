package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumDrive extends OpMode {
        DcMotor RFMotor;
        DcMotor LFMotor;
        DcMotor RBMotor;
        DcMotor LBMotor;


        @Override
        public void init() {
            RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
            LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
            RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
            LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

            RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        @Override
        public void loop(){
            moveDriveTrain();
        }

        public void moveDriveTrain() {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            double fl = y - x - rx;
            double bl = y + x - rx;
            double fr = y + x + rx;
            double br = y - x + rx;

            LFMotor.setPower(fl*0.5);
            LBMotor.setPower(bl*0.5);
            RFMotor.setPower(fr*0.5);
            RBMotor.setPower(br*0.5);

        }
}
