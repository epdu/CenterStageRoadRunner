package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawCodeIncrements extends OpMode {
    public Servo ClawServo;
    double HalfOpen = .25; //should be 45 degrees of rotation- first pixel
    double Open = .5; //should be 90 degrees- second pixel
    @Override
    public void init() { ClawServo = hardwareMap.get(Servo.class, "claw");}
    @Override
    public void loop() {
        if (gamepad1.a) {
            ClawServo.setPosition(HalfOpen);}

        if (gamepad1.b) {
            ClawServo.setPosition(Open);}

        if (gamepad1.x) {
            ClawServo.setPosition(0);}    //Return to closed
        }
    }

