package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ActiveIntake extends OpMode {
    DcMotor Intake;
    public Servo IntakeServo;
    boolean move = false;
    public float speedMultiplier = 1;
    public float speedMultiplier1 = 0.2f;


    @Override
    public void init () {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
    }


    @Override
    public void loop(){
        moveIntake();
        if (gamepad2.b && !move) {
            IntakeServo.setPosition(0.5);
        }else if (gamepad2.x && !move){
            IntakeServo.setPosition(0);
        }else {
            ejectPixel();
        }
    }

    public void moveIntake(){
        double intake = gamepad2.right_trigger;
        Intake.setPower(intake*speedMultiplier);
    }

    public void ejectPixel(){
        double intake = gamepad2.left_trigger;
        Intake.setPower(intake*speedMultiplier1);
    }

}
