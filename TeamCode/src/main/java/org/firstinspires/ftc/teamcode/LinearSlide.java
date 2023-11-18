package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlide extends OpMode {
    DcMotor liftMotor;





    @Override
    public void init() {

    }

    @Override
    public void loop() {
    double y = - gamepad1.left_stick_y;
    liftMotor.setPower(0.5 * y);
    }
}
