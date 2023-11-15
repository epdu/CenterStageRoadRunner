package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Suspensioncode extends OpMode {

    private Servo servo;
    public boolean previous1 = false;
    public boolean previous2 = false;

    private DcMotorEx motor;


    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "Servo");
        motor = hardwareMap.get(DcMotorEx.class, "Motor");
        servo.setPosition(0.5);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        moveServo(gamepad1.dpad_up, gamepad1.dpad_down);

        motor.setPower(gamepad1.left_stick_y);


        telemetry.addData("SERVO", servo.getPosition());
        telemetry.update();
    }

    public void moveServo(boolean keybind1, boolean keybind2)
    {
        boolean current1 = keybind1;
        if(current1 && !previous1)
        {
            servo.setPosition(servo.getPosition() + 0.1);
        }
        previous1 = current1;

        boolean current2 = keybind2;
        if(current2 && !previous2)
        {
            servo.setPosition(servo.getPosition() - 0.1);
        }
        previous2=current2;
    }
}
