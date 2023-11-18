package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ActiveIntake extends OpMode {
    DcMotor Intake;
    boolean move = false;


    @Override
    public void init (){
        Intake = hardwareMap.get(DcMotor.class, "Intake");
    }

    @Override
    public void loop(){
        if (gamepad1.y && !move) {
        moveIntake
        }
    }

    public void moveIntake(){
        Intake = gamepad1
    }
}
