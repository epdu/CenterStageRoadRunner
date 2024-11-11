package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class LauncherServo extends OpMode {
    public Servo LauncherServo;
    @Override
    public void init() {
        LauncherServo = hardwareMap.get(Servo.class, "LauncherServo");
    }
    @Override
    public void loop() {
        if (gamepad2.x) {
            LauncherServo.setPosition(0.25);}
    }
}

