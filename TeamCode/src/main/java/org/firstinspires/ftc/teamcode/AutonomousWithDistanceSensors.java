package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.Date;
import java.io.FileWriter;
import java.io.File;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.opencv.*;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import com.qualcomm.robotcore.util.RobotLog;
import java.lang.Math;
import java.util.Locale;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.Files;
import android.os.Environment;




@Autonomous
public class AutonomousWithDistanceSensors extends LinearOpMode {

    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
 //    Blinker control_Hub;
 //   DcMotor lift;
    DistanceSensor LeftSensor;
    DistanceSensor RightSensor;
    IMU imu;
//    BNO055IMU imu;
    //    Servo clawLeft;
//    Servo clawRight;
    double ticksPerRotation;
    double initialFR;
    double initialFL;
    double initialBR;
    double initialBL;
    double liftInitial;
    double positionFR;
    double positionFL;
    double positionBR;
    double positionBL;
    public String updates;
    public int i = 0;
    double targetheading;
    double heading;
    double liftIdealPos;
    double liftIdealPower;
    double previousHeading;
    double processedHeading;
    double redVal;
    double blueVal;
    double greenVal;
    int result;
    double  distanceInInch;
    double  distanceInInchDouble;
    private double wheelDiameterInInches = 3.77953;  // Adjust this based on your mecanum wheel diameter



    // @Override
    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        LeftSensor = hardwareMap.get(DistanceSensor.class, "DistanceLeft");
        RightSensor = hardwareMap.get(DistanceSensor.class, "DistanceRight");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
    //left side motors
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    // right side motors
/*
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
*/
        // ticks per revolution
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

// set the distanct from frot of robot to the block of game element
/*  Using the specs from the motor, you would need to find the encoder counts per revolution (of the output shaft).
     Then, you know that corresponds to 360 degrees of wheel rotation, which means the distance travelled is the circumference
      of the wheel (2 * pi * r_wheel). To figure out how many encoder ticks correspond to the distance you wanna go,
      just multiply the distance by the counts / distance you calculated above. Hope that helps!
// 11.87374348
//537 per revolution 11.87374348 inch
*/
        distanceInInch=24;//number in unit of inch
        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
/*        imu.initialize(parameters);
        moveForward(0.2, distanceInInchDouble);
        sleep(1000);
        moveBackward(0.2, distanceInInchDouble);
        sleep(1000);

 */
        findPixel();
//        droppixel();

      /*  StrafingRight(0.2, 1000);
        sleep(1000);//strafing left
        AprilTagOmni(); //find the right april tag and approach it
        droppixelbackdrop();
        goparking();

        moveForward(0.5, 1000);
        sleep(2000);
        sleep(2000);

        RightTurn(0.2, 500);
        sleep(1000);
        LeftTurn(0.2, 500);
        sleep(1000);
        StrafingLeft(0.2, 1000);
        sleep(1000);//strafing left
*/

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftReading = LeftSensor.getDistance(DistanceUnit.INCH);
            double rightReading = RightSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            telemetry.update();
        }

    }
    // Adjust the orientation parameters to match your robot
    public void gyroTurn(double power, double degrees){ //right is negative
        if(opModeIsActive()){
            double gyroinitial = newGetHeading();
            if(degrees>0){ //turn left
                while(newGetHeading() - gyroinitial < degrees && opModeIsActive()){
                    RFMotor.setPower(power);
                    LBMotor.setPower(-power);
                    LFMotor.setPower(-power);
                    RBMotor.setPower(power);


                    updates = Double.toString(newGetHeading());
                    telemetry.addData("Heading", newGetHeading());
                    telemetry.update();
                }
                stopMotors();
            }
            else{//turn right
                while(newGetHeading() - gyroinitial > degrees && opModeIsActive()){
                    RFMotor.setPower(-power);
                    LBMotor.setPower(power);
                    LFMotor.setPower(power);
                    RBMotor.setPower(-power);

                    updates = Double.toString(getHeading());
                    telemetry.addData("Heading", newGetHeading());
                    telemetry.update();
                }
                stopMotors();
            }
        }
    }

    public void absoluteHeading(double power, double degrees){
//As you can see, this only works if you also have the newGetHeading() and gyroTurn() functions.
//gyroTurn() is where the loop is - where it would lock people out - so you might need
//to copy all three functions and make changes to gyroTurn().
//newGetHeading() should probably not cause any problems, though.
        double current = newGetHeading(); //Set the current heading to what the heading is,accounting for
//the whole -179 -> 180 flip
        double processedCurrent = current % 360.0;//The processed version of the current heading
//(This just removes extra rotations)
        telemetry.addData("how many to turn",degrees-processedCurrent);
        telemetry.addData("power", power); //We probably don't need any of these telemetry things
        telemetry.update(); //But here they are
        gyroTurn(power,degrees-processedCurrent); //This is where the actual turning bit happens.
//It uses gyroTurn(), which you'll probably have to adapt for teleop use.
    }
    public void goStraight(double power,double rotations){
        double multiplier;
        initialFR = RFMotor.getCurrentPosition()/ticksPerRotation;
        initialFL = LFMotor.getCurrentPosition()/ticksPerRotation;
        initialBR = RBMotor.getCurrentPosition()/ticksPerRotation;
        initialBL = LBMotor.getCurrentPosition()/ticksPerRotation;
        positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
        positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
        positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
        positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
        RFMotor.setPower(power);
        LFMotor.setPower(power);
        RBMotor.setPower(power);
        LBMotor.setPower(power);
        targetheading = getHeading();
        while(positionFR < rotations && positionFL < rotations && positionBR < rotations &&
                positionBL < rotations && opModeIsActive()){
            heading = getHeading();
            if(heading-targetheading>=0){ //to the left
                multiplier = .1*(heading-targetheading)+1;
                LFMotor.setPower(power*multiplier);
                LBMotor.setPower(power*multiplier);
                RFMotor.setPower(power);
                RBMotor.setPower(power);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFMotor.setPower(power*multiplier);
                RBMotor.setPower(power*multiplier);
                LFMotor.setPower(power);
                LBMotor.setPower(power);
            }
            positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
            positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
            positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
            positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
        }
        stopMotors();
    }

    public void findPixel(){
        double power = .25;
        double multiplier;
        targetheading = getHeading();
        double leftReading = LeftSensor.getDistance(DistanceUnit.INCH);
        double rightReading = RightSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Left", leftReading);
        telemetry.addData("Right", rightReading);
        telemetry.update();
/*
        if(leftReading > 32 && rightReading > 32 || leftReading < 10 || rightReading < 10){
//if the cone is too far out, give up
            return;
        }
 */

        int i = 0;
        long timeBeforeLoop = System.currentTimeMillis();
        leftReading = LeftSensor.getDistance(DistanceUnit.INCH);
        rightReading = RightSensor.getDistance(DistanceUnit.INCH);
        RobotLog.aa("ConeRadar", "testing");
        while(Math.abs(leftReading - rightReading) > .5){
            double heading = getHeading();
//first, fix the left-right error
            if(Math.abs(leftReading - rightReading) > 2){
                power = .25;
            }else{
                power = .15;
            }
            if(leftReading > rightReading){ //if we need to go right
                heading = getHeading();
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(power*multiplier);
                    LBMotor.setPower(-power);
                    RFMotor.setPower(-power*multiplier);
                    RBMotor.setPower(power);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    LFMotor.setPower(power);
                    LBMotor.setPower(-power*multiplier);
                    RFMotor.setPower(-power);
                    RBMotor.setPower(power*multiplier);

                }
            }else{ //if we need to go left
                heading = getHeading();
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(-power);
                    LBMotor.setPower(power*multiplier);
                    RFMotor.setPower(power);
                    RBMotor.setPower(-power*multiplier);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(power*multiplier);
                    RBMotor.setPower(-power);
                    LFMotor.setPower(-power*multiplier);
                    LBMotor.setPower(power);
                }
            }
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " +
                    Double.toString(rightReading));
            telemetry.update();
            leftReading = LeftSensor.getDistance(DistanceUnit.INCH);
            rightReading = RightSensor.getDistance(DistanceUnit.INCH);
            i++;
        } //end of the left-right error loop
        RobotLog.aa("NOTE", "strafing part finished");
        while(leftReading > 5.5 && rightReading > 5.5){ //go forward
            leftReading = LeftSensor.getDistance(DistanceUnit.INCH);
            rightReading = RightSensor.getDistance(DistanceUnit.INCH);
            heading = getHeading();
            if(leftReading < 8 || rightReading < 8){
                power = .15;
            }else{
                power = .25;
            }
            if(heading-targetheading>=0){ //to the left
                multiplier = .1*(heading-targetheading)+1;
                LFMotor.setPower(power*multiplier);
                LBMotor.setPower(power*multiplier);
                RFMotor.setPower(power);
                RBMotor.setPower(power);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFMotor.setPower(power*multiplier);
                RBMotor.setPower(power*multiplier);
                LFMotor.setPower(power);
                LBMotor.setPower(power);
            }
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " +
                    Double.toString(rightReading));
            telemetry.update();
        }
        while(Math.abs(leftReading - rightReading) > .5){
            double heading = getHeading();
//first, fix the left-right error
            power = .15;
            if(leftReading > rightReading){ //if we need to go right
                heading = getHeading();
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(power*multiplier);
                    LBMotor.setPower(-power);
                    RFMotor.setPower(-power*multiplier);
                    RBMotor.setPower(power);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(-power);
                    RBMotor.setPower(power*multiplier);
                    LFMotor.setPower(power);
                    LBMotor.setPower(-power*multiplier);
                }
            }else{ //if we need to go left
                heading = getHeading();
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(-power);
                    LBMotor.setPower(power*multiplier);
                    RFMotor.setPower(power);
                    RBMotor.setPower(-power*multiplier);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(power*multiplier);
                    RBMotor.setPower(-power);
                    LFMotor.setPower(-power*multiplier);
                    LBMotor.setPower(power);
                }
            }
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " +
                    Double.toString(rightReading));
            telemetry.update();
            leftReading = LeftSensor.getDistance(DistanceUnit.INCH);
            rightReading = RightSensor.getDistance(DistanceUnit.INCH);
            i++;
        } //end of the left-right error loop
        stopMotors();
    }

    public void goBackward(double power,double rotations){
        double multiplier;
        double intended = rotations * -1;
        initialFR = RFMotor.getCurrentPosition()/ticksPerRotation;
        initialFL = LFMotor.getCurrentPosition()/ticksPerRotation;
        initialBR = RBMotor.getCurrentPosition()/ticksPerRotation;
        initialBL = LBMotor.getCurrentPosition()/ticksPerRotation;
        positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
        positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
        positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
        positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
        RFMotor.setPower(-power);
        LFMotor.setPower(-power);
        RBMotor.setPower(-power);
        LBMotor.setPower(-power);
        targetheading = getHeading();
        while(positionFR > intended && positionFL > intended && positionBR > intended &&
                positionBL > intended && opModeIsActive()){
            heading = getHeading();
            if(heading-targetheading>=0){ //to the left
                multiplier = .1*(heading-targetheading)+1;
                LFMotor.setPower(-power);
                LBMotor.setPower(-power);
                RFMotor.setPower(-power*multiplier);
                RBMotor.setPower(-power*multiplier);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFMotor.setPower(-power);
                RBMotor.setPower(-power);
                LFMotor.setPower(-power*multiplier);
                LBMotor.setPower(-power*multiplier);
            }
            positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
            positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
            positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
            positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
        }
        stopMotors();
    }
    public void strafeRight(double power, double rotations, double timelimit){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        String frpower;
        String flpower;
        String brpower;
        String blpower;
        initialFR = RFMotor.getCurrentPosition()/ticksPerRotation;
        initialFL = LFMotor.getCurrentPosition()/ticksPerRotation;
        initialBR = RBMotor.getCurrentPosition()/ticksPerRotation;
        initialBL = LBMotor.getCurrentPosition()/ticksPerRotation;
        positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
        positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
        positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
        positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
        RFMotor.setPower(-power);
        LFMotor.setPower(power);
        RBMotor.setPower(power);
        LBMotor.setPower(-power);
        targetheading = getHeading();
        while(positionFR > -rotations && positionFL<rotations && positionBR < rotations &&
                positionBL > -rotations && current-startthing < 1000*timelimit && opModeIsActive()){
            heading = getHeading();
            frpower = Double.toString(RFMotor.getPower());
            flpower = Double.toString(LFMotor.getPower());
            brpower = Double.toString(RBMotor.getPower());
            blpower = Double.toString(LBMotor.getPower());
/* telemetry.addData("Front Right", " " + frpower);
telemetry.addLine();
telemetry.addData("Front Left", " " + flpower);
telemetry.addLine();
telemetry.addData("Back Right", " " + brpower);
telemetry.addLine();
telemetry.addData("Back Left", " " + blpower);
telemetry.addLine();
telemetry.addData("Heading", " " + Double.toString(heading));
telemetry.update();*/
            if(heading-targetheading>=0){
                multiplier = .1*(heading-targetheading)+1;
                LFMotor.setPower(power*multiplier);
                LBMotor.setPower(-power);
                RFMotor.setPower(-power*multiplier);
                RBMotor.setPower(power);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFMotor.setPower(-power);
                RBMotor.setPower(power*multiplier);
                LFMotor.setPower(power);
                LBMotor.setPower(-power*multiplier);
            }
            positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
            positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
            positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
            positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
            current = System.currentTimeMillis();
        }
        stopMotors();
    }
    public void strafeLeft(double power, double rotations, double timelimit){
        long startthing = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        double multiplier;
        String frpower;
        String flpower;
        String brpower;
        String blpower;
        initialFR = RFMotor.getCurrentPosition()/ticksPerRotation;
        initialFL = LFMotor.getCurrentPosition()/ticksPerRotation;
        initialBR = RBMotor.getCurrentPosition()/ticksPerRotation;
        initialBL = LBMotor.getCurrentPosition()/ticksPerRotation;
        positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
        positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
        positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
        positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
        RFMotor.setPower(power);
        LFMotor.setPower(-power);
        RBMotor.setPower(-power);
        LBMotor.setPower(power);
        targetheading = getHeading();
        while(positionFR < rotations && positionFL > -rotations && positionBR > -rotations &&
                positionBL < rotations && current - startthing < 1000*timelimit && opModeIsActive()){
            heading = getHeading();
            frpower = Double.toString(RFMotor.getPower());
            flpower = Double.toString(LFMotor.getPower());
            brpower = Double.toString(RBMotor.getPower());
            blpower = Double.toString(LBMotor.getPower());
            if(heading-targetheading>=0){
                multiplier = .1*(heading-targetheading)+1;
                LFMotor.setPower(-power);
                LBMotor.setPower(power*multiplier);
                RFMotor.setPower(power);
                RBMotor.setPower(-power*multiplier);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFMotor.setPower(power*multiplier);
                RBMotor.setPower(-power);
                LFMotor.setPower(-power*multiplier);
                LBMotor.setPower(power);
            }
            positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
            positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
            positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
            positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
            current = System.currentTimeMillis();
        }
        stopMotors();
    }

    public void stopMotors(){
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }
 /*
    public void setLiftLevel(double level){
        double position = (lift.getCurrentPosition()/ticksPerRotation)-liftInitial;
        double idealPosition = 0;
        if(level == 1){
            idealPosition = .48;
        }
        else if(level == 2){
            idealPosition = .74;
        }
        else if (level == 3){
            idealPosition = 1.06;
        }else if(level == 0){
            idealPosition = -.01;
        }
//Slides position: 0.46 for low goal, 0.74 for middle goal, 1.06 for high goal
//Move lift to ground, level 1, level 2, level 3
        while(Math.abs(position - idealPosition) > 0.02 && opModeIsActive()){
            position = (lift.getCurrentPosition()/ticksPerRotation)-liftInitial;
//used to be while
            if (position > idealPosition){
                lift.setPower(-0.25);
            }
            if (position < idealPosition){
                lift.setPower(0.5); // used to be 0.25, that was slow
            }
            if(Math.abs(position-idealPosition) < 0.02){
                lift.setPower(0.01);
            }
        }
    }

*/



    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

/*
//// Create an object to receive the IMU angles
//YawPitchRollAngles robotOrientation;
//robotOrientation = imu.getRobotYawPitchRollAngles();
//
//// Now use these simple methods to extract each angle
//// (Java type double) from the object you just created:
//double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
//double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
//double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

// Create Orientation variable
        Orientation myRobotOrientation;

// Get Robot Orientation
        myRobotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        );

// Then read or display the desired values (Java type float):
        float X_axis = myRobotOrientation.firstAngle;
        float Y_axis = myRobotOrientation.secondAngle;
        float Z_axis = myRobotOrientation.thirdAngle;

//        BNO055IMU
//        imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
FIRST Tech Challenge robots drive mostly on a flat playing field, typically using the IMU to monitor or control Heading (Yaw or Z-angle).

*/
    }
    public double newGetHeading(){
        double currentHeading =imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
// BNO055IMU
//double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
/*

Orientation 	getAngularOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit)
Returns the absolute orientation of the sensor as a set three angles with indicated parameters.
*
*
*
*/
        double headingChange = currentHeading - previousHeading;
        if(headingChange < -180){
            headingChange += 360;
        }else if(headingChange > 180){
            headingChange -= 360;
        }
        processedHeading += headingChange;
        previousHeading = currentHeading;
        return processedHeading;
    }
    private void goparking() {
    }

    private void AprilTagOmni() {
    }

    private void droppixelbackdrop() {
    }
    public void droppixel(){}
    //
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
        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {}
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
        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {}
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


        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {}
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


        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {}
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


        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {}
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


        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {}
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }


}


