package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class servogobrrrr extends OpMode {
    Servo ArmR;
    Servo ArmL;
    boolean move = false;
    @Override
    public void init() {
        ArmL = hardwareMap.get(Servo.class, "ArmL");
        ArmR = hardwareMap.get(Servo.class, "ArmR");

    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up && !move) { //up
            ArmR.setPosition(0);
            ArmL.setPosition(0);
        }
        if (gamepad2.dpad_down && !move) { //down
            ArmL.setPosition(0.8);
            ArmR.setPosition(0.8);
        }
    }
}
