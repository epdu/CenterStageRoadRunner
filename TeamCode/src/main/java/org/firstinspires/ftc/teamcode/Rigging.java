package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Rigging extends OpMode {
    DcMotor RightRig;
    DcMotor LeftRig;

    @Override
    public void init() {
        RightRig = hardwareMap.get(DcMotor.class, "RightRig");
        LeftRig = hardwareMap.get(DcMotor.class, "LeftRig");
        RightRig.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

    }
}
