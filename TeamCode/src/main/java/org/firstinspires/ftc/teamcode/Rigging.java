package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Rigging extends OpMode {
    DcMotor RightRig;
    DcMotor LeftRig;
    boolean move = false;

    @Override
    public void init() {
        RightRig = hardwareMap.get(DcMotor.class, "RightRig");
        LeftRig = hardwareMap.get(DcMotor.class, "LeftRig");
        RightRig.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop() {
    if (gamepad2.x && !move){
        RightRig.setTargetPosition(100);
        RightRig.setPower(1);
        LeftRig.setTargetPosition(100);
        LeftRig.setPower(1);}
    if (gamepad2.b && !move){
        RightRig.setTargetPosition(0);
        RightRig.setPower(0);
        LeftRig.setTargetPosition(0);
        LeftRig.setPower(0);}
    }
}
