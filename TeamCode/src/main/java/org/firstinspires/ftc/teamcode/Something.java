package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class Something extends OpMode {
    DistanceSensor distanceSensor;

    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distance sensor");
    }

    @Override
    public void loop() {
    double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
    telemetry.addData( "Distance", distanceInches );
telemetry.update();
    }


}
