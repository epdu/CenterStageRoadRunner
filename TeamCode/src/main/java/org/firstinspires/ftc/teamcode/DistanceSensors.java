package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous
    public class DistanceSensors extends LinearOpMode {
    DistanceSensor LeftSensor;
    DistanceSensor RightSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftSensor = hardwareMap.get(DistanceSensor.class, "DistanceLeft");
        RightSensor = hardwareMap.get(DistanceSensor.class, "DistanceRight");
    }

}