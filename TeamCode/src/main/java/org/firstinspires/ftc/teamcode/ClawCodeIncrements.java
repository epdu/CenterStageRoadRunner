package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp

public class ClawCodeIncrements extends OpMode {
    public Servo ClawServo;
    double HalfOpen = .55; //should be first pixel
    double Open = .57; //should be second pixel

    @Override
    public void init() { ClawServo = hardwareMap.get(Servo.class, "claw");}
    @Override
    public void loop() {
        if (gamepad1.a) {
            ClawServo.setPosition(HalfOpen);}

        if (gamepad1.b) {
            ClawServo.setPosition(Open);}

        if (gamepad1.x) {
            ClawServo.setPosition(0.5);}    //Return to closed
        }
    }

