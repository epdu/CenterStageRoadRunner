package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class AtiveW extends LinearOpMode {

    DcMotor TopMotor;

@Override public void runOpMode()  {
        TopMotor = hardwareMap.dcMotor.get("TopMotor"); //hub port
        // RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
            ActiveIn();
        }
    }
    public void ActiveIn(){
        double TopMotory = gamepad1.left_stick_y;
        TopMotor.setPower(TopMotory);
        TopMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
