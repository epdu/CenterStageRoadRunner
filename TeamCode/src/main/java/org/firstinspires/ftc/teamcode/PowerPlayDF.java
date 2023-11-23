package org.firstinspires.ftc.teamcode;
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
public class PowerPlayDF extends LinearOpMode{
    Blinker control_Hub;
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    DcMotor lift;
    DistanceSensor distanceLeft;
    DistanceSensor distanceRight;
    BNO055IMU imu;
    Servo clawLeft;
    Servo clawRight;
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
    // @Override
    public void runOpMode() {
//Claw positions:
//closed is .85 left, 0 right
//open is .65 left, .3 right
        initializeHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
// Wait for the game to start (driver presses PLAY)
        waitForStart();
        initialFR = RFMotor.getCurrentPosition()/ticksPerRotation;
        initialFL = LFMotor.getCurrentPosition()/ticksPerRotation;
        initialBR = RBMotor.getCurrentPosition()/ticksPerRotation;
        initialBL = LBMotor.getCurrentPosition()/ticksPerRotation;


        
        findCone();
// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            double rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            telemetry.update();
        }
    }
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
                    RFmotor.setPower(-power);
                    motorBL.setPower(power);
                    motorFL.setPower(power);
                    motorBR.setPower(-power);
                    
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
        initialFR = RFmotor.getCurrentPosition()/ticksPerRotation;
        initialFL = motorFL.getCurrentPosition()/ticksPerRotation;
        initialBR = motorBR.getCurrentPosition()/ticksPerRotation;
        initialBL = motorBL.getCurrentPosition()/ticksPerRotation;
        positionFR = (RFmotor.getCurrentPosition()/ticksPerRotation)-initialFR;
        positionFL = (motorFL.getCurrentPosition()/ticksPerRotation)-initialFL;
        positionBR = (motorBR.getCurrentPosition()/ticksPerRotation)-initialBR;
        positionBL = (motorBL.getCurrentPosition()/ticksPerRotation)-initialBL;
        RFmotor.setPower(power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(power);
        targetheading = getHeading();
        while(positionFR < rotations && positionFL < rotations && positionBR < rotations &&
                positionBL < rotations && opModeIsActive()){
            heading = getHeading();
            if(heading-targetheading>=0){ //to the left
                multiplier = .1*(heading-targetheading)+1;
                motorFL.setPower(power*multiplier);
                motorBL.setPower(power*multiplier);
                RFmotor.setPower(power);
                motorBR.setPower(power);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFmotor.setPower(power*multiplier);
                motorBR.setPower(power*multiplier);
                motorFL.setPower(power);
                motorBL.setPower(power);
            }
            positionFR = (RFmotor.getCurrentPosition()/ticksPerRotation)-initialFR;
            positionFL = (motorFL.getCurrentPosition()/ticksPerRotation)-initialFL;
            positionBR = (motorBR.getCurrentPosition()/ticksPerRotation)-initialBR;
            positionBL = (motorBL.getCurrentPosition()/ticksPerRotation)-initialBL;
        }
        stopMotors();
    }
    public void findCone(){
        double power = .25;
        double multiplier;
        targetheading = getHeading();
        double leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
        double rightReading = distanceRight.getDistance(DistanceUnit.INCH);
        telemetry.addData("Left", leftReading);
        telemetry.addData("Right", rightReading);
        telemetry.update();
        if(leftReading > 17 && rightReading > 17 || leftReading < 5 || rightReading < 5){
//if the cone is too far out, give up
            return;
        }
        int i = 0;
        long timeBeforeLoop = System.currentTimeMillis();
        leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
        rightReading = distanceRight.getDistance(DistanceUnit.INCH);
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
                    motorFL.setPower(power*multiplier);
                    motorBL.setPower(-power);
                    RFmotor.setPower(-power*multiplier);
                    motorBR.setPower(power);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFmotor.setPower(-power);
                    motorBR.setPower(power*multiplier);
                    motorFL.setPower(power);
                    motorBL.setPower(-power*multiplier);
                }
            }else{ //if we need to go left
                heading = getHeading();
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    motorFL.setPower(-power);
                    motorBL.setPower(power*multiplier);
                    RFmotor.setPower(power);
                    motorBR.setPower(-power*multiplier);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFmotor.setPower(power*multiplier);
                    motorBR.setPower(-power);
                    motorFL.setPower(-power*multiplier);
                    motorBL.setPower(power);
                }
            }
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " +
                    Double.toString(rightReading));
            telemetry.update();
            leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            i++;
        } //end of the left-right error loop
        RobotLog.aa("NOTE", "strafing part finished");
        while(leftReading > 5.5 && rightReading > 5.5){ //go forward
            leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            heading = getHeading();
            if(leftReading < 8 || rightReading < 8){
                power = .15;
            }else{
                power = .25;
            }
            if(heading-targetheading>=0){ //to the left
                multiplier = .1*(heading-targetheading)+1;
                motorFL.setPower(power*multiplier);
                motorBL.setPower(power*multiplier);
                RFmotor.setPower(power);
                motorBR.setPower(power);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFmotor.setPower(power*multiplier);
                motorBR.setPower(power*multiplier);
                motorFL.setPower(power);
                motorBL.setPower(power);
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
                    motorFL.setPower(power*multiplier);
                    motorBL.setPower(-power);
                    RFmotor.setPower(-power*multiplier);
                    motorBR.setPower(power);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFmotor.setPower(-power);
                    motorBR.setPower(power*multiplier);
                    motorFL.setPower(power);
                    motorBL.setPower(-power*multiplier);
                }
            }else{ //if we need to go left
                heading = getHeading();
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    motorFL.setPower(-power);
                    motorBL.setPower(power*multiplier);
                    RFmotor.setPower(power);
                    motorBR.setPower(-power*multiplier);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFmotor.setPower(power*multiplier);
                    motorBR.setPower(-power);
                    motorFL.setPower(-power*multiplier);
                    motorBL.setPower(power);
                }
            }
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " +
                    Double.toString(rightReading));
            telemetry.update();
            leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            i++;
        } //end of the left-right error loop
        stopMotors();
    }
    public void goBackward(double power,double rotations){
        double multiplier;
        double intended = rotations * -1;
        initialFR = RFmotor.getCurrentPosition()/ticksPerRotation;
        initialFL = motorFL.getCurrentPosition()/ticksPerRotation;
        initialBR = motorBR.getCurrentPosition()/ticksPerRotation;
        initialBL = motorBL.getCurrentPosition()/ticksPerRotation;
        positionFR = (RFmotor.getCurrentPosition()/ticksPerRotation)-initialFR;
        positionFL = (motorFL.getCurrentPosition()/ticksPerRotation)-initialFL;
        positionBR = (motorBR.getCurrentPosition()/ticksPerRotation)-initialBR;
        positionBL = (motorBL.getCurrentPosition()/ticksPerRotation)-initialBL;
        RFmotor.setPower(-power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(-power);
        targetheading = getHeading();
        while(positionFR > intended && positionFL > intended && positionBR > intended &&
                positionBL > intended && opModeIsActive()){
            heading = getHeading();
            if(heading-targetheading>=0){ //to the left
                multiplier = .1*(heading-targetheading)+1;
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                RFmotor.setPower(-power*multiplier);
                motorBR.setPower(-power*multiplier);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFmotor.setPower(-power);
                motorBR.setPower(-power);
                motorFL.setPower(-power*multiplier);
                motorBL.setPower(-power*multiplier);
            }
            positionFR = (RFmotor.getCurrentPosition()/ticksPerRotation)-initialFR;
            positionFL = (motorFL.getCurrentPosition()/ticksPerRotation)-initialFL;
            positionBR = (motorBR.getCurrentPosition()/ticksPerRotation)-initialBR;
            positionBL = (motorBL.getCurrentPosition()/ticksPerRotation)-initialBL;
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
        initialFR = RFmotor.getCurrentPosition()/ticksPerRotation;
        initialFL = motorFL.getCurrentPosition()/ticksPerRotation;
        initialBR = motorBR.getCurrentPosition()/ticksPerRotation;
        initialBL = motorBL.getCurrentPosition()/ticksPerRotation;
        positionFR = (RFmotor.getCurrentPosition()/ticksPerRotation)-initialFR;
        positionFL = (motorFL.getCurrentPosition()/ticksPerRotation)-initialFL;
        positionBR = (motorBR.getCurrentPosition()/ticksPerRotation)-initialBR;
        positionBL = (motorBL.getCurrentPosition()/ticksPerRotation)-initialBL;
        RFmotor.setPower(-power);
        motorFL.setPower(power);
        motorBR.setPower(power);
        motorBL.setPower(-power);
        targetheading = getHeading();
        while(positionFR > -rotations && positionFL<rotations && positionBR < rotations &&
                positionBL > -rotations && current-startthing < 1000*timelimit && opModeIsActive()){
            heading = getHeading();
            frpower = Double.toString(RFmotor.getPower());
            flpower = Double.toString(motorFL.getPower());
            brpower = Double.toString(motorBR.getPower());
            blpower = Double.toString(motorBL.getPower());
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
                motorFL.setPower(power*multiplier);
                motorBL.setPower(-power);
                RFmotor.setPower(-power*multiplier);
                motorBR.setPower(power);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFmotor.setPower(-power);
                motorBR.setPower(power*multiplier);
                motorFL.setPower(power);
                motorBL.setPower(-power*multiplier);
            }
            positionFR = (RFmotor.getCurrentPosition()/ticksPerRotation)-initialFR;
            positionFL = (motorFL.getCurrentPosition()/ticksPerRotation)-initialFL;
            positionBR = (motorBR.getCurrentPosition()/ticksPerRotation)-initialBR;
            positionBL = (motorBL.getCurrentPosition()/ticksPerRotation)-initialBL;
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
        initialFR = RFmotor.getCurrentPosition()/ticksPerRotation;
        initialFL = motorFL.getCurrentPosition()/ticksPerRotation;
        initialBR = motorBR.getCurrentPosition()/ticksPerRotation;
        initialBL = motorBL.getCurrentPosition()/ticksPerRotation;
        positionFR = (RFmotor.getCurrentPosition()/ticksPerRotation)-initialFR;
        positionFL = (motorFL.getCurrentPosition()/ticksPerRotation)-initialFL;
        positionBR = (motorBR.getCurrentPosition()/ticksPerRotation)-initialBR;
        positionBL = (motorBL.getCurrentPosition()/ticksPerRotation)-initialBL;
        RFmotor.setPower(power);
        motorFL.setPower(-power);
        motorBR.setPower(-power);
        motorBL.setPower(power);
        targetheading = getHeading();
        while(positionFR < rotations && positionFL > -rotations && positionBR > -rotations &&
                positionBL < rotations && current - startthing < 1000*timelimit && opModeIsActive()){
            heading = getHeading();
            frpower = Double.toString(RFmotor.getPower());
            flpower = Double.toString(motorFL.getPower());
            brpower = Double.toString(motorBR.getPower());
            blpower = Double.toString(motorBL.getPower());
            if(heading-targetheading>=0){
                multiplier = .1*(heading-targetheading)+1;
                motorFL.setPower(-power);
                motorBL.setPower(power*multiplier);
                RFmotor.setPower(power);
                motorBR.setPower(-power*multiplier);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFmotor.setPower(power*multiplier);
                motorBR.setPower(-power);
                motorFL.setPower(-power*multiplier);
                motorBL.setPower(power);
            }
            positionFR = (RFmotor.getCurrentPosition()/ticksPerRotation)-initialFR;
            positionFL = (motorFL.getCurrentPosition()/ticksPerRotation)-initialFL;
            positionBR = (motorBR.getCurrentPosition()/ticksPerRotation)-initialBR;
            positionBL = (motorBL.getCurrentPosition()/ticksPerRotation)-initialBL;
            current = System.currentTimeMillis();
        }
        stopMotors();
    }
    public void stopMotors(){
        RFmotor.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }
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
    public double getHeading(){
        return
                imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
    }
    public double newGetHeading(){
        double currentHeading =
                imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
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
    //closed is .85 left, 0 right
//open is .65 left, .3 right
    public void openClaw(){
        clawLeft.setPosition(.65);
        clawRight.setPosition(.7);
    }
    public void closeClaw(){
        clawLeft.setPosition(.9);
        clawRight.setPosition(.3);
    }
    public void initializeHardware(){
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        RFmotor = hardwareMap.get(DcMotor.class, "motor0");
        motorFL = hardwareMap.get(DcMotor.class, "motor1");
        motorBR = hardwareMap.get(DcMotor.class, "motor2");
        motorBL = hardwareMap.get(DcMotor.class, "motor3");
        lift = hardwareMap.get(DcMotor.class, "lift");
        clawLeft = hardwareMap.get(Servo.class, "clawRight");
        clawRight = hardwareMap.get(Servo.class, "clawLeft");
// left is port 0, right is port 2
        distanceLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        ticksPerRotation = RFmotor.getMotorType().getTicksPerRev();
        initialFR = RFmotor.getCurrentPosition()/ticksPerRotation;
        initialFL = motorFL.getCurrentPosition()/ticksPerRotation;
        initialBR = motorBR.getCurrentPosition()/ticksPerRotation;
        initialBL = motorBL.getCurrentPosition()/ticksPerRotation;
        liftInitial = (lift.getCurrentPosition()/ticksPerRotation);
/* motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
motorBL.setDirection(DcMotorSimple.Direction.REVERSE);*/
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
//Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal &
                0x0F);
        sleep(100); //Changing modes requires a delay before doing anything else
//Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
//Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
//Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100); //Changing modes again requires a delay
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    /*
    ------------------COMPUTER VISION STUFF IS DOWN HERE---------------------------
    */
    public void doCV() {
// initializeHardware();
        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera =
                OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
// Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
//telemetry.addData("Result", getResult());
/* sleep(500);
camera.stopStreaming(); */
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        EmptyPipeline myPipeline = new EmptyPipeline();
        camera.setPipeline(myPipeline);
//EmptyPipeline yourPipeline;
// telemetry.addData("Status", "Initialized");
// telemetry.update();
// Wait for the game to start (driver presses PLAY)
        waitForStart();
// run until the end of the match (driver presses STOP)
    }
    class EmptyPipeline extends OpenCvPipeline
    {
        @Override
        public Mat processFrame(Mat input)
        {
//red 1 green 2 blue 3
            Mat signal = input.submat(new Rect(new Point(380, 200), new Point(430, 300)));
//coordinate system has origin in top left corner because they wanted to make things hard. ok.
//so (800, 800) is bottom right corner.
//(800, 0) is top right corner.
//(0, 800) is bottom left corner. we can do this.
//x-coordinates are what most people would make them.
//y-coordinates are the opposite.
                    Mat green = new Mat();
            Mat red = new Mat();
            Mat blue = new Mat();
            Core.extractChannel(signal, green, 1);
            Core.extractChannel(signal, red, 0);
            Core.extractChannel(signal, blue, 2);
            greenVal = Core.mean(green).val[0];
            redVal = Core.mean(red).val[0];
            blueVal = Core.mean(blue).val[0];
            signal.release();
            green.release();
            red.release();
            blue.release();
/*telemetry.addData("Webcam says", "");
telemetry.addData("red", redVal);
telemetry.addData("green", greenVal);
telemetry.addData("blue", blueVal);*/
/* if(redVal > blueVal && redVal > greenVal){
// telemetry.addData("Color", "red");
result = 1;
}else if(blueVal > redVal && blueVal > greenVal){
//telemetry.addData("Color", "blue");
result = 2;
}else{
// telemetry.addData("Color", "green");
result = 3;
}*/
// telemetry.update();
            return signal;
        }
    }
    public int getResult(){
        return result;
    }
}
