package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;
import java.lang.Long;
import java.lang.Double;


@Autonomous
public class AutonomousCode extends LinearOpMode {

    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    double  distanceInInch;
    double  distanceInInchDouble;
    private double wheelDiameterInInches = 3.77953;  // Adjust this based on your mecanum wheel diameter

    @Override
    public void runOpMode() throws InterruptedException {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ticks per revolution
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        distanceInInch=24;//number in unit of inch
//      distanceInInchDouble=distanceInInch.doubleValue();
        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));

        // 11.87374348


        //537 per revolution 11.87374348 inch
        moveForward(0.2, distanceInInchDouble);
        sleep(2000);
        moveBackward(0.2, distanceInInchDouble);
        sleep(2000);
        RightTurn(0.2, distanceInInchDouble);
        sleep(1000);
        LeftTurn(0.2, distanceInInchDouble);
        sleep(1000);
        StrafingLeft(0.2, distanceInInchDouble);
        sleep(1000);//strafing left
        StrafingRight(0.2, distanceInInchDouble);
        sleep(1000);//strafing left

    }
    //test function]
    public void moveForward(double power, double distance) {
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setTargetPosition((int) -distance);
        RBMotor.setTargetPosition((int) -distance);
        LFMotor.setTargetPosition((int) -distance);
        LBMotor.setTargetPosition((int) -distance);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setPower(+power);
        RBMotor.setPower(+power);
        LFMotor.setPower(+power);
        LBMotor.setPower(+power);
        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {
            // Output telemetry data
            telemetry.addData("LF Position", LFMotor.getCurrentPosition());
            telemetry.addData("RF Position", RFMotor.getCurrentPosition());
            telemetry.addData("LB Position", LBMotor.getCurrentPosition());
            telemetry.addData("RB Position", RBMotor.getCurrentPosition());
            telemetry.addData("getTicksPerRev()", LFMotor.getMotorType().getTicksPerRev());
            telemetry.update();
        }
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }
    public void moveBackward(double power, double distance) {
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setTargetPosition((int) distance);
        RBMotor.setTargetPosition((int) distance);
        LFMotor.setTargetPosition((int) distance);
        LBMotor.setTargetPosition((int) distance);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setPower(+power);
        RBMotor.setPower(+power);
        LFMotor.setPower(+power);
        LBMotor.setPower(+power);
        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {            // Output telemetry data
            telemetry.addData("LF Position", LFMotor.getCurrentPosition());
            telemetry.addData("RF Position", RFMotor.getCurrentPosition());
            telemetry.addData("LB Position", LBMotor.getCurrentPosition());
            telemetry.addData("RB Position", RBMotor.getCurrentPosition());
            telemetry.addData("getTicksPerRev()", LFMotor.getMotorType().getTicksPerRev());
            telemetry.update();}
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }

    public void RightTurn(double power, double distance) {
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setTargetPosition((int) -distance);
        LBMotor.setTargetPosition((int) -distance);
        RFMotor.setTargetPosition((int) +distance);
        RBMotor.setTargetPosition((int) +distance);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setPower(+power);
        LBMotor.setPower(+power);
        RFMotor.setPower(+power);
        RBMotor.setPower(+power);


        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {            // Output telemetry data
            telemetry.addData("LF Position", LFMotor.getCurrentPosition());
            telemetry.addData("RF Position", RFMotor.getCurrentPosition());
            telemetry.addData("LB Position", LBMotor.getCurrentPosition());
            telemetry.addData("RB Position", RBMotor.getCurrentPosition());
            telemetry.addData("getTicksPerRev()", LFMotor.getMotorType().getTicksPerRev());
            telemetry.update();}
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }
    public void LeftTurn(double power, double distance) {
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setTargetPosition((int) +distance);
        LBMotor.setTargetPosition((int) +distance);
        RFMotor.setTargetPosition((int) -distance);
        RBMotor.setTargetPosition((int) -distance);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setPower(+power);
        LBMotor.setPower(+power);
        RFMotor.setPower(+power);
        RBMotor.setPower(+power);


        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {            // Output telemetry data
            telemetry.addData("LF Position", LFMotor.getCurrentPosition());
            telemetry.addData("RF Position", RFMotor.getCurrentPosition());
            telemetry.addData("LB Position", LBMotor.getCurrentPosition());
            telemetry.addData("RB Position", RBMotor.getCurrentPosition());
            telemetry.addData("getTicksPerRev()", LFMotor.getMotorType().getTicksPerRev());
            telemetry.update();}
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }


    public void StrafingLeft(double power, double distance) {
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setTargetPosition((int) +distance);
        LBMotor.setTargetPosition((int) -distance);
        RFMotor.setTargetPosition((int) -distance);
        RBMotor.setTargetPosition((int) +distance);



        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setPower(+power);
        LBMotor.setPower(+power);
        RFMotor.setPower(+power);
        RBMotor.setPower(+power);


        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {            // Output telemetry data
            telemetry.addData("LF Position", LFMotor.getCurrentPosition());
            telemetry.addData("RF Position", RFMotor.getCurrentPosition());
            telemetry.addData("LB Position", LBMotor.getCurrentPosition());
            telemetry.addData("RB Position", RBMotor.getCurrentPosition());
            telemetry.addData("getTicksPerRev()", LFMotor.getMotorType().getTicksPerRev());
            telemetry.update();}
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }
    public void StrafingRight(double power, double distance) {
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setTargetPosition((int) -distance);
        LBMotor.setTargetPosition((int) +distance);
        RFMotor.setTargetPosition((int) +distance);
        RBMotor.setTargetPosition((int) -distance);



        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setPower(+power);
        LBMotor.setPower(+power);
        RFMotor.setPower(+power);
        RBMotor.setPower(+power);


        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {            // Output telemetry data
            telemetry.addData("LF Position", LFMotor.getCurrentPosition());
            telemetry.addData("RF Position", RFMotor.getCurrentPosition());
            telemetry.addData("LB Position", LBMotor.getCurrentPosition());
            telemetry.addData("RB Position", RBMotor.getCurrentPosition());
            telemetry.addData("getTicksPerRev()", LFMotor.getMotorType().getTicksPerRev());
            telemetry.update();}
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }


}


