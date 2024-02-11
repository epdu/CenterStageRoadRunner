package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmTesting extends OpMode {
    Servo ArmR;
    Servo ArmL;
    boolean move = false;
    @Override
    public void init() {
        ArmL = hardwareMap.get(Servo.class, "ArmL");
        ArmR = hardwareMap.get(Servo.class, "ArmR");
        ArmL.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_down && !move) { //down
            ArmR.setPosition(0);
            ArmL.setPosition(0);
        }
        if (gamepad2.dpad_up && !move) { //up
            ArmL.setPosition(0.9);
            ArmR.setPosition(0.9);
        }
    }
}
