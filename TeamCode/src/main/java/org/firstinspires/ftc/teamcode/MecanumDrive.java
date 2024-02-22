package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumDrive extends OpMode {
        DcMotor RFMotor;
        DcMotor LFMotor;
        DcMotor RBMotor;
        DcMotor LBMotor;
        public Servo Drone;
        boolean move = false;


        @Override
        public void init() {
            RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
            LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
            RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
            LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

            RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            Drone = hardwareMap.get(Servo.class, "Drone");
            Drone.setPosition(0);
        }
        @Override
        public void loop(){
            moveDriveTrain();
            if (gamepad2.left_bumper && !move) { //shoot
                Drone.setPosition(1);
            }
        }

        public void moveDriveTrain() {
            double y = gamepad1.left_stick_y*0.8;
            double x =- gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x*0.5;


            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;
//            double y = gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//            double fl = y - x - rx;
//            double bl = y + x - rx;
//            double fr = y + x + rx;
//            double br = y - x + rx;

            LFMotor.setPower(fl);
            LBMotor.setPower(bl);
            RFMotor.setPower(fr);
            RBMotor.setPower(br);

        }
}
