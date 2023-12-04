package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous
    public class DistanceSensors extends LinearOpMode {
    DistanceSensor LeftSensor;
    DistanceSensor RightSensor;
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftSensor = hardwareMap.get(DistanceSensor.class, "DistanceLeft");
        RightSensor = hardwareMap.get(DistanceSensor.class, "DistanceRight");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");


    }
}