package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ActiveIntake extends OpMode {
    DcMotor Intake;
    boolean move = false;
    public float speedMultiplier = 1;
    public float speedMultiplier1 = 0.2f;


    @Override
    public void init (){
        Intake = hardwareMap.get(DcMotor.class, "Intake");
    }

    @Override
    public void loop(){
        if (gamepad1.y && !move) {
        moveIntake();
        }
    }

    public void moveIntake(){
        double intake = gamepad1.right_trigger;
        Intake.setPower(intake*speedMultiplier);
    }

}
