package org.firstinspires.ftc.teamcode;
import java.util.Date;
import com.qualcomm.robotcore.hardware.Blinker;
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
import org.opencv.imgproc.Imgproc;
import com.qualcomm.robotcore.util.RobotLog;
import java.lang.Math;
import java.util.Locale;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
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
    DistanceSensor poleLeft;
    DistanceSensor poleRight;
    BNO055IMU imu;
    Servo clawLeft;
    Servo clawRight;
    Servo arm;
    Servo wrist;
    double ticksPerRotation;
    double initialFR;
    double initialFL;
    double initialBR;
    double initialBL;
    double liftInitial;
    double positionFR;
    boolean goForPole = true;
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
    int cycleNumber = 0;
    double clawLeftOpen = 0.38;
    double clawLeftClosed = 0.75;
    double clawRightOpen = 0.51;
    double clawRightClosed = .14;
    double armFront = 0.439;//.03;
    double armBack = 0.6675;//.245;
    double wristNormal = .98;
    double wristFlipped = .3175;
    boolean hasFirstConeBeenDelivered = false;
    /*
    BL = 0.95
    BR = 0.95
    FL = 0.956250
    FR = 1
    */
    double multiplierBL = 0.95; //0.93686; //(3443.0000000/3780.0000000);
    double multiplierBR = 0.95;   //1;// 1;
    double multiplierFR = 1; //1;// 3443.0000000/3677.0000000;
    double multiplierFL = 0.956250; //0.956250; // 3443.0000000/3822.0000000;
    //Slides position: 0.46 for low goal, 0.74 for middle goal, 1.06 for high goal
    //.1, .42, .7

    double liftLevelOne = .12;
    double liftLevelTwo = .43; //.41
    double liftLevelTwoPreload = .46;
    double liftLevelThree = .8; //.77
    double liftLevelFour = .14;
    double liftLevelFive = .11;
    double liftLevelSix = .07;
    //liftLevel 4 - 8 will be for having 5 (4), 4 (5), 3 (6), 2 (7), or 1 (8) cones left on the stack
//    @Override
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
    public void strafeToPole(int isL){ //1 is left side of field, -1 is right
        double stoppingPoint;
        if(hasFirstConeBeenDelivered == false){ // first cone
            stoppingPoint = .25; //.1875
        }
        else{ // later cones
            stoppingPoint = .4;
        }
        boolean rightHasSeenIt = false;
        boolean leftHasSeenIt = false;
        if(isL == 1){
            RFMotor.setPower(.3*multiplierFR);
            LBMotor.setPower(.3*multiplierBL);
            RBMotor.setPower(-.3*multiplierBR);
            LFMotor.setPower(-.3*multiplierFL);
            double currentH = getHeading();
            RobotLog.aa("Heading", Double.toString(currentH));
            double target = 90.0;
            double encoderInitial = RFMotor.getCurrentPosition() / ticksPerRotation;
            double encoderCurrent = RFMotor.getCurrentPosition() / ticksPerRotation;
            goForPole = true;
            double leftForStrafe = poleLeft.getDistance(DistanceUnit.INCH);
            double rightForStrafe = poleRight.getDistance(DistanceUnit.INCH);
            RobotLog.aa("Strafe to pole initial, Sensors", leftForStrafe + ", " +  rightForStrafe);
            while((!rightHasSeenIt || !leftHasSeenIt) && leftForStrafe > 0.0 && rightForStrafe > 0.0 && opModeIsActive()){
                liftWithinLoop();
                if(currentH-target>=0){
                    double multiplier = .05*(currentH-target)+1;
                    LFMotor.setPower(-.3*multiplierFL);
                    LBMotor.setPower(.3 * multiplier*multiplierBL);
                    RFMotor.setPower(.3*multiplierFR);
                    RBMotor.setPower(-.3 * multiplier*multiplierBR);
                    RobotLog.aa("Multiplier", Double.toString(multiplier));
                }else if(currentH-target<0){
                    double multiplier = -.05*(currentH-target)+1;
                    RFMotor.setPower(.3 * multiplier*multiplierFR);
                    RBMotor.setPower(-.3*multiplierBR);
                    LFMotor.setPower(-.3 * multiplier*multiplierFL);
                    LBMotor.setPower(.3*multiplierBL);
                    RobotLog.aa("Multiplier", Double.toString(multiplier));
                }
                encoderCurrent = RFMotor.getCurrentPosition() / ticksPerRotation;
                if((encoderCurrent - encoderInitial) > stoppingPoint){
                    // StrafeToPole has failed at this point
                    RobotLog.aa("progress", "StrafeToPole failed, we're trying to strafe back");
                    goForPole = false;
              /*  if(hasFirstConeBeenDelivered == true){ // for later cones
                    if(isLeft == 1){
                        strafeRight(.35, .22, 5); // .1 wors fine, but not dead center
                    }else{
                        strafeLeft(.35, .22, 5);
                    }
                }
                else{ // for first cone
                    if(isLeft == 1){
                        strafeRight(.35, .135, 5); // .125 works fine, but not dead center
                    }else{
                        strafeLeft(.35, .135, 5);
                    }
                }
                break;*/
                    bailAndPark(isL, result, cycleNumber);
                }
                leftForStrafe = poleLeft.getDistance(DistanceUnit.INCH);
                rightForStrafe = poleRight.getDistance(DistanceUnit.INCH);
                if(leftForStrafe < 18){
                    leftHasSeenIt = true;
                }
                if(rightForStrafe < 18){
                    rightHasSeenIt = true;
                }
                // DELETE THE TWO LINES BELOW. THIS WILL CAUSE STRAFETOPOLE TO FAIL EVERY RUN.
                // REMEMBER TO DELETE THESE TWO LINES.
           /*   leftForStrafe = 20;
              rightForStrafe = 20;*/
                RobotLog.aa("Strafe to pole, Sensors", leftForStrafe + ", " + rightForStrafe);
                currentH = getHeading();
            }
            stopMotors();
        }else{
            RFMotor.setPower(-.3*multiplierFR);
            LBMotor.setPower(-.3*multiplierBL);
            RBMotor.setPower(.3*multiplierBR);
            LFMotor.setPower(.3*multiplierFL);
            double currentH = getHeading();
            RobotLog.aa("Heading", Double.toString(currentH));
            double target = -90.0;
            double encoderInitial = RBMotor.getCurrentPosition() / ticksPerRotation;
            double encoderCurrent = RBMotor.getCurrentPosition() / ticksPerRotation;
            goForPole = true;
            double leftForStrafe = poleLeft.getDistance(DistanceUnit.INCH);
            double rightForStrafe = poleRight.getDistance(DistanceUnit.INCH);
            RobotLog.aa("Strafe to pole initial, Sensors", leftForStrafe + ", " +  rightForStrafe);
            while((!rightHasSeenIt || !leftHasSeenIt) && leftForStrafe > 0.0 && rightForStrafe > 0.0 && opModeIsActive()){
                liftWithinLoop();
                if(currentH-target>=0){
                    double multiplier = .05*(currentH-target)+1;
                    LFMotor.setPower(.3 * multiplier*multiplierFL);
                    LBMotor.setPower(-.3*multiplierBL);
                    RFMotor.setPower(-.3 * multiplier*multiplierFR);
                    RBMotor.setPower(.3*multiplierBR);
                    RobotLog.aa("Multiplier", Double.toString(multiplier));
                }else if(currentH-target<0){
                    double multiplier = -.05*(currentH-target)+1;
                    RFMotor.setPower(-.3*multiplierFR);
                    RBMotor.setPower(.3 * multiplier*multiplierBR);
                    LFMotor.setPower(.3*multiplierFL);
                    LBMotor.setPower(-.3 * multiplier*multiplierBL);
                    RobotLog.aa("Multiplier", Double.toString(multiplier));
                }
                encoderCurrent = RBMotor.getCurrentPosition() / ticksPerRotation;
                RobotLog.aa("give-up encoder readings: ", Double.toString(encoderCurrent), Double.toString(encoderInitial), Double.toString(stoppingPoint));
                if((encoderCurrent - encoderInitial) > stoppingPoint){
                    // StrafeToPole has failed at this point
                    RobotLog.aa("progress", "StrafeToPole failed, we're trying to strafe back");
                    goForPole = false;
               /* if(hasFirstConeBeenDelivered == true){ // for later cones
                    if(isLeft == 1){
                        strafeRight(.35, .22, 5); // .1 wors fine, but not dead center
                    }else{
                        strafeLeft(.35, .22, 5);
                    }
                }
                else{ // for first cone
                    if(isLeft == 1){
                        strafeRight(.35, .135, 5); // .125 works fine, but not dead center
                    }else{
                        strafeLeft(.35, .135, 5);
                    }
                }
                break;*/
                    bailAndPark(isL, result, cycleNumber);
                }
                leftForStrafe = poleLeft.getDistance(DistanceUnit.INCH);
                rightForStrafe = poleRight.getDistance(DistanceUnit.INCH);
                if(leftForStrafe < 18){
                    leftHasSeenIt = true;
                }
                if(rightForStrafe < 18){
                    rightHasSeenIt = true;
                }
                // DELETE THE TWO LINES BELOW. THIS WILL CAUSE STRAFETOPOLE TO FAIL EVERY RUN.
                // REMEMBER TO DELETE THESE TWO LINES.
                //  leftForStrafe = 20;
                //  rightForStrafe = 20;
                RobotLog.aa("Strafe to pole, Sensors", leftForStrafe + ", " + rightForStrafe);
                currentH = getHeading();
            }
            stopMotors();
        }
        hasFirstConeBeenDelivered = true;
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
                    liftWithinLoop();
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
                    liftWithinLoop();
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
        double current = newGetHeading(); //Set the current heading to what the heading is, accounting for
        //the whole -179 -> 180 flip
        double processedCurrent = current % 360.0;//The processed version of the current heading
        //(This just removes extra rotations)
        telemetry.addData("how many to turn",Double.toString(degrees-processedCurrent));
        RobotLog.aa("Degrees", Double.toString(degrees));
        RobotLog.aa("ProcessedCurrent", Double.toString(processedCurrent));
        RobotLog.aa("AbsoluteHeadingShouldTurn", Double.toString(degrees-processedCurrent));
        telemetry.addData("power", power); //We probably don't need any of these telemetry things
        telemetry.update(); //But here they are
        gyroTurn(power,degrees-processedCurrent); //This is where the actual turning bit happens.
        //It uses gyroTurn(), which you'll probably have to adapt for teleop use.
    }
    public void goStraight(double power, double rotations, double idealHeading){
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
        targetheading = idealHeading;
        RobotLog.aa("GoStraight", "goal heading is " + Double.toString(targetheading));
        while(positionFR < rotations && positionFL < rotations && positionBR < rotations && positionBL < rotations && opModeIsActive()){
            liftWithinLoop();
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
    public void goStraight(double power, double rotations){
        goStraight(power, rotations, getHeading());
    }
    public void findCone(){
        double power = .25;
        double multiplier;
        targetheading = getHeading();
        RobotLog.aa("FindCone", "goal heading is " + Double.toString(targetheading));
        double leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
        double rightReading = distanceRight.getDistance(DistanceUnit.INCH);
        if(leftReading < 5){
            leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            if(leftReading < 5){
                leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            }
        }
        if(rightReading < 5){
            rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            if(rightReading < 5){
                rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            }
        }
        telemetry.addData("Left", leftReading);
        telemetry.addData("Right", rightReading);
        telemetry.update();
        RobotLog.aa("ConeRadar", "sensor initial values are " + Double.toString(leftReading) + ", " + Double.toString(rightReading));
        if(leftReading > 21 && rightReading > 21 || leftReading < 5 || rightReading < 5){
            //if the cone is too far out, give up
            return;
        }
        int i = 0;
        boolean strafingRight = false;
        long timeBeforeLoop = System.currentTimeMillis();
        leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
        rightReading = distanceRight.getDistance(DistanceUnit.INCH);
        RobotLog.aa("ConeRadar", "testing");
        while(Math.abs(leftReading - rightReading) > .75){
            liftWithinLoop();
            double heading = getHeading();
            //first, fix the left-right error
            if(Math.abs(leftReading - rightReading) > 2){
                power = .25;
            }else{
                power = .15;
            }
            if(i == 0){
                strafingRight = leftReading > rightReading;
            }else{
                if(strafingRight != leftReading > rightReading){
                    break;
                }
            }
            if(leftReading > rightReading){ //if we need to go right
                heading = getHeading();

                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(power*multiplier*multiplierFL);
                    LBMotor.setPower(-power*multiplierBL);
                    RFMotor.setPower(-power*multiplier*multiplierFR);
                    RBMotor.setPower(power*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(-power*multiplierFR);
                    RBMotor.setPower(power*multiplier*multiplierBR);
                    LFMotor.setPower(power*multiplierFL);
                    LBMotor.setPower(-power*multiplier*multiplierBL);
                }
            }else{ //if we need to go left
                heading = getHeading();
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(-power*multiplierFL);
                    LBMotor.setPower(power*multiplier*multiplierBL);
                    RFMotor.setPower(power*multiplierFR);
                    RBMotor.setPower(-power*multiplier*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(power*multiplier*multiplierFR);
                    RBMotor.setPower(-power*multiplierBR);
                    LFMotor.setPower(-power*multiplier*multiplierFL);
                    LBMotor.setPower(power*multiplierBL);
                }
            }
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " + Double.toString(rightReading));
            telemetry.update();
            leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            i++;
        } //end of the left-right error
        RobotLog.aa("NOTE", "strafing part finished");
        while(leftReading > 5.5 && rightReading > 5.5){ //go forward
            liftWithinLoop();
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
                LFMotor.setPower(power*multiplier*multiplierFL);
                LBMotor.setPower(power*multiplier*multiplierBL);
                RFMotor.setPower(power*multiplierFR);
                RBMotor.setPower(power*multiplierBR);
            }else if(heading-targetheading<0){
                multiplier = -.1*(heading-targetheading)+1;
                RFMotor.setPower(power*multiplier*multiplierFR);
                RBMotor.setPower(power*multiplier*multiplierBR);
                LFMotor.setPower(power*multiplierFL);
                LBMotor.setPower(power*multiplierBL);
            }
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " + Double.toString(rightReading));
            telemetry.update();
        }
        int j = 0;
        strafingRight = false;

        while(Math.abs(leftReading - rightReading) > .5){
            liftWithinLoop();
            double heading = getHeading();
            //first, fix the left-right error
            power = .15;
            if(j == 0){
                strafingRight = leftReading > rightReading;
            }else{
                if(strafingRight != leftReading > rightReading){
                    break;
                }
            }
            if(leftReading > rightReading){ //if we need to go right
                heading = getHeading();
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(power*multiplier*multiplierFL);
                    LBMotor.setPower(-power*multiplierBL);
                    RFMotor.setPower(-power*multiplier*multiplierFR);
                    RBMotor.setPower(power*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(-power*multiplierFR);
                    RBMotor.setPower(power*multiplier*multiplierBR);
                    LFMotor.setPower(power*multiplierFL);
                    LBMotor.setPower(-power*multiplier*multiplierBL);
                }
            }else{ //if we need to go left
                heading = getHeading();
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(-power*multiplierFL);
                    LBMotor.setPower(power*multiplier*multiplierBL);
                    RFMotor.setPower(power*multiplierFR);
                    RBMotor.setPower(-power*multiplier*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(power*multiplier*multiplierFR);
                    RBMotor.setPower(-power*multiplierBR);
                    LFMotor.setPower(-power*multiplier*multiplierFL);
                    LBMotor.setPower(power*multiplierBL);
                }
            }
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " + Double.toString(rightReading));
            telemetry.update();
            leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            i++;
        } //end of the left-right error loop

        stopMotors();
    }

    public void goBackward(double power,double rotations, double idealHeading){
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
        targetheading = idealHeading;
        RobotLog.aa("GoBackward", "goal heading is " + Double.toString(targetheading));
        while(positionFR > intended && positionFL > intended && positionBR > intended && positionBL > intended && opModeIsActive()){
            heading = getHeading();
            liftWithinLoop();
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
    public void goBackward(double power, double rotations){
        goBackward(power, rotations, getHeading());
    }
    public void strafeRight(double power, double rotations, double timelimit, double idealHeading){
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
        targetheading = idealHeading;
        RobotLog.aa("StrafeRight", "goal heading is " + Double.toString(targetheading));
        while(positionFR > -rotations && positionFL<rotations && positionBR < rotations && positionBL > -rotations && current-startthing < 1000*timelimit && opModeIsActive()){
            heading = getHeading();
            liftWithinLoop();
            frpower = Double.toString(RFMotor.getPower());
            flpower = Double.toString(LFMotor.getPower());
            brpower = Double.toString(RBMotor.getPower());
            blpower = Double.toString(LBMotor.getPower());
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
    public void strafeRight(double power, double rotations, double timeLimit){
        strafeRight(power, rotations, timeLimit, getHeading());
    }
    public void strafeLeft(double power, double rotations, double timelimit, double idealHeading){
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
        targetheading = idealHeading;
        RobotLog.aa("StrafeLeft", "goal heading is " + Double.toString(targetheading));
        while(positionFR < rotations && positionFL > -rotations && positionBR > -rotations && positionBL < rotations && current - startthing < 1000*timelimit && opModeIsActive()){
            heading = getHeading();
            liftWithinLoop();
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
    public void strafeLeft(double power, double rotations, double timeLimit){
        strafeLeft(power, rotations, timeLimit, getHeading());
    }
    public void stopMotors(){
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }
    public void setLiftLevel(double level){
        double position = (lift.getCurrentPosition()/ticksPerRotation)-liftInitial;
        double idealPosition = 0;
        if(level == 1){
            idealPosition = liftLevelOne;
        }
        else if(level == 2){
            idealPosition = liftLevelTwo;
        }
        else if (level == 3){
            idealPosition = liftLevelThree;
        }else if(level == 0){
            idealPosition = -.01;
        }else if(level == 4){
            idealPosition = liftLevelFour;
        }else if(level == 5){
            idealPosition = liftLevelFive;
        }
        liftIdealPos = idealPosition;

        //Slides position: 0.46 for low goal, 0.74 for middle goal, 1.06 for high goal
        //Move lift to ground, level 1, level 2, level 3
        while(Math.abs(position - idealPosition) > 0.02 && opModeIsActive()){
            position = (lift.getCurrentPosition()/ticksPerRotation)-liftInitial;
            //used to be while
            if (position > idealPosition){
                lift.setPower(-0.35);
            }
            if (position < idealPosition){
                lift.setPower(0.7); // used to be 0.25, that was slow
            }
            if(Math.abs(position-idealPosition) < 0.02){
                if(level == 0){
                    lift.setPower(0);
                }else if(level == 1 || level == 2){
                    lift.setPower(.15);
                }else if(level == 3){
                    lift.setPower(.2);
                }
                /*
                if (slidesPos > 0.1 && slidesPos < 0.74){
                        //So that it doesn't pop back up when it hits the bottom
                        slides.setPower(0.15);
                        //drivePower = 0.75;
                    }
                    else if (slidesPos > 0.70){
                        //at the top, it needs more power to keep it up
                        slides.setPower(0.2);
                        //drivePower = 0.4;
                    }
                    else if (slidesPos < 0.1){
                        slides.setPower(0);
                        //drivePower = 0.75;
                    }
                */
                //lift.setPower(0.05);
            }
        }
    }

    public void liftForTime(double power, double time){
        lift.setPower(power);
        long start = System.currentTimeMillis();
        long current = System.currentTimeMillis();
        while(current - start < time && opModeIsActive()){
            current = System.currentTimeMillis();
        }
        stopMotors();
    }
    public double getHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
    }
    public double newGetHeading(){
        double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
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
        clawLeft.setPosition(clawLeftOpen);
        clawRight.setPosition(clawRightOpen);
    }
    public void closeClaw(){
        clawLeft.setPosition(clawLeftClosed);
        clawRight.setPosition(clawRightClosed);
    }
    public void liftWithinLoop(){
        double slidesPos = lift.getCurrentPosition()/ticksPerRotation - liftInitial;
        if(Math.abs(slidesPos - liftIdealPos) < .02){
            if (slidesPos > 0.1 && slidesPos < 0.74){
                //So that it doesn't pop back up when it hits the bottom
                lift.setPower(0.15);
                //drivePower = 0.75;
            }
            else if (slidesPos > 0.70){
                //at the top, it needs more power to keep it up
                lift.setPower(0.2);
                //drivePower = 0.4;
            }
            else if (slidesPos < 0.1){
                lift.setPower(0);
                //drivePower = 0.75;
            }

        }else{
            if(slidesPos < liftIdealPos){
                if(liftIdealPos - slidesPos > .1){
                    lift.setPower(.5);
                }else{
                    lift.setPower(.35);
                }
            }else{
                lift.setPower(-.25);
            }
        }
    }
    public void findPole(){
        double power = .25;
        double multiplier;
        targetheading = getHeading();
        RobotLog.aa("FindPole", "goal heading is " + Double.toString(targetheading));
        double leftReading = poleLeft.getDistance(DistanceUnit.INCH);
        double rightReading = poleRight.getDistance(DistanceUnit.INCH);
        double goodLeftValue = leftReading;
        double goodRightValue = rightReading;
        telemetry.addData("Left", leftReading);
        telemetry.addData("Right", rightReading);
        telemetry.update();
        RobotLog.aa("Initial sensor readings", Double.toString(leftReading) + ", " + Double.toString(rightReading));
        if(leftReading > 17 && rightReading > 17 || leftReading < 2 || rightReading < 2){
            //if the cone is too far out, give up
            return;
        }
        int i = 0;
        long timeBeforeLoop = System.currentTimeMillis();
        RobotLog.aa("PoleRadar", "before initial strafe");
        boolean strafingRight = false;
        while(Math.abs(leftReading - rightReading) > .75){
            liftWithinLoop();
            double heading = getHeading();
            //first, fix the left-right error
            if(Math.abs(leftReading - rightReading) > 2){
                power = .25;
            }else{
                power = .15;
            }
            if(i == 0){
                strafingRight = leftReading > rightReading;
            }else{
                if(strafingRight != leftReading > rightReading){
                    break;
                }
            }
            if(leftReading > rightReading){ //if we need to go right
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(power*multiplier*multiplierFL);
                    LBMotor.setPower(-power*multiplierBL);
                    RFMotor.setPower(-power*multiplier*multiplierFR);
                    RBMotor.setPower(power*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(-power*multiplierFR);
                    RBMotor.setPower(power*multiplier*multiplierBR);
                    LFMotor.setPower(power*multiplierFL);
                    LBMotor.setPower(-power*multiplier*multiplierBL);
                }
            }else{ //if we need to go left
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(-power*multiplierFL);
                    LBMotor.setPower(power*multiplier*multiplierBL);
                    RFMotor.setPower(power*multiplierFR);
                    RBMotor.setPower(-power*multiplier*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(power*multiplier*multiplierFR);
                    RBMotor.setPower(-power*multiplierBR);
                    LFMotor.setPower(-power*multiplier*multiplierFL);
                    LBMotor.setPower(power*multiplierBL);
                }
            }
            leftReading = poleLeft.getDistance(DistanceUnit.INCH);
            rightReading = poleRight.getDistance(DistanceUnit.INCH);
            if(leftReading < 322){
                goodLeftValue = leftReading;
            }
            if(rightReading < 322){
                goodRightValue = rightReading;
            }
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            telemetry.update();
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " + Double.toString(rightReading));
            i++;
        } //end of the left-right error loop
        RobotLog.aa("NOTE", "initial strafe finished");
        RobotLog.aa("GoodSensors", Double.toString(goodLeftValue) + ", " + Double.toString(goodRightValue));
        if(leftReading > 300){
            leftReading = goodLeftValue;
        }
        if(rightReading > 300){
            rightReading = goodRightValue;
        }
        if(leftReading > 7.5 && rightReading > 7.5){
            RobotLog.aa("Forwards", "is where we're going");
            while(leftReading > 7.5 && rightReading > 7.5){ //go forward
                heading = getHeading();
                liftWithinLoop();
                if(leftReading < 9 || rightReading < 9){
                    power = -.15;
                }else{
                    power = -.25;
                }
                if(heading-targetheading>=0){ //to the left
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(power*multiplier*multiplierFL);
                    LBMotor.setPower(power*multiplier*multiplierBL);
                    RFMotor.setPower(power*multiplierFR);
                    RBMotor.setPower(power*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(power*multiplier*multiplierFR);
                    RBMotor.setPower(power*multiplier*multiplierBR);
                    LFMotor.setPower(power*multiplierFL);
                    LBMotor.setPower(power*multiplierBL);
                }
                leftReading = poleLeft.getDistance(DistanceUnit.INCH);
                rightReading = poleRight.getDistance(DistanceUnit.INCH);
                telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
                telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
                RobotLog.aa("Sensors", Double.toString(leftReading) + ", " + Double.toString(rightReading));
                telemetry.update();
            }
        }
        else if((leftReading < 6.5 || rightReading < 6.5) && (leftReading != 0.0 && rightReading != 0.0)){
            RobotLog.aa("Backwards", "in pole radar");
            while(leftReading < 6.5 || rightReading < 6.5 && (leftReading != 0.0 && rightReading != 0.0)){ //go forward
                heading = getHeading();
                liftWithinLoop();
                if(leftReading > 5 || rightReading > 5){
                    power = .15;
                }else{
                    power = .25;
                }
                if(heading-targetheading>=0){ //to the left
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(power*multiplier*multiplierFL);
                    LBMotor.setPower(power*multiplier*multiplierBL);
                    RFMotor.setPower(power*multiplierFR);
                    RBMotor.setPower(power*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(power*multiplier*multiplierFR);
                    RBMotor.setPower(power*multiplier*multiplierBR);
                    LFMotor.setPower(power*multiplierFL);
                    LBMotor.setPower(power*multiplierBL);
                }
                leftReading = poleLeft.getDistance(DistanceUnit.INCH);
                rightReading = poleRight.getDistance(DistanceUnit.INCH);
                telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
                telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
                RobotLog.aa("Sensors", Double.toString(leftReading) + ", " + Double.toString(rightReading));
                telemetry.update();
            }
        }
        int j = 0;
        strafingRight = false;
        RobotLog.aa("NOTE", "backwards/forwards finished, starting final strafe");
        while(Math.abs(leftReading - rightReading) > .5){
            liftWithinLoop();
            double heading = getHeading();
            //first, fix the left-right error
            power = .18; //0.15
            if(j == 0){
                strafingRight = leftReading > rightReading;
            }else{
                if(strafingRight != leftReading > rightReading){
                    break;
                }
            }
            if(leftReading > rightReading){ //if we need to go right
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(power*multiplier*multiplierFL);
                    LBMotor.setPower(-power*multiplierBL);
                    RFMotor.setPower(-power*multiplier*multiplierFR);
                    RBMotor.setPower(power*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(-power*multiplierFR);
                    RBMotor.setPower(power*multiplier*multiplierBR);
                    LFMotor.setPower(power*multiplierFL);
                    LBMotor.setPower(-power*multiplier*multiplierBL);
                }
            }else{ //if we need to go left
                if(heading-targetheading>=0){
                    multiplier = .1*(heading-targetheading)+1;
                    LFMotor.setPower(-power*multiplierFL);
                    LBMotor.setPower(power*multiplier*multiplierBL);
                    RFMotor.setPower(power*multiplierFR);
                    RBMotor.setPower(-power*multiplier*multiplierBR);
                }else if(heading-targetheading<0){
                    multiplier = -.1*(heading-targetheading)+1;
                    RFMotor.setPower(power*multiplier*multiplierFR);
                    RBMotor.setPower(-power*multiplierBR);
                    LFMotor.setPower(-power*multiplier*multiplierFL);
                    LBMotor.setPower(power*multiplierBL);
                }
            }
            leftReading = poleLeft.getDistance(DistanceUnit.INCH);
            rightReading = poleRight.getDistance(DistanceUnit.INCH);
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            telemetry.update();
            RobotLog.aa("Sensors", Double.toString(leftReading) + ", " + Double.toString(rightReading));
            j++;
        } //end of the left-right error loop
        RobotLog.aa("NOTE", "PoleRadar finished");

        stopMotors();
    }
    public void bailAndPark(int IL, int result, int cycleNumber){
        if(cycleNumber == 4){
            absoluteHeading(.4, 180.0 * IL);
            absoluteHeading(.2, 180.0 * IL);
            wrist.setPosition(wristNormal);
            arm.setPosition(armFront);
            goBackward(.3, .1, 180.0 * IL);
            absoluteHeading(.2, 180.0 * IL);
            sleep(25000);
        }else{
            wrist.setPosition(wristNormal);
            arm.setPosition(armFront);
            if(result == 1){
                if(IL == 1){
                    goStraight(.7, .35);
                }else{
                    goBackward(.7, .33); //.38
                }
            }else if(result == 3){
                if(IL == 1){
                    goBackward(.7, .29); //.34
                }else{
                    goStraight(.7, .42); //.37
                }
            }else{

            }
            liftIdealPos = -.01;
            absoluteHeading(.4,180.0 * IL);
            absoluteHeading(.2, 180.0 * IL);
            goBackward(.3, .1, 180.0 * IL);
            absoluteHeading(.2, 180.0 * IL);
            sleep(25000);
        }
        /*
    wrist.setPosition(wristNormal);
    arm.setPosition(armFront);
    if(result == 1){
      if(isLeft == 1){
        goStraight(.7, .3);
      }else{
        goBackward(.7, .38);
      }
    }else if(result == 3){
      if(isLeft == 1){
        goBackward(.7, .34);
      }else{
        goStraight(.7, .37);
      }
    }else{

    }
    liftIdealPos = -.01;
    absoluteHeading(.4,180.0 * isLeft);
    absoluteHeading(.2, 180.0 * isLeft);
    goBackward(.3, .1, 180.0 * isLeft);
    absoluteHeading(.2, 180.0 * isLeft);
    */
    }
    public void initializeHardware(){
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        RFMotor = hardwareMap.get(DcMotor.class, "motor0");
        LFMotor = hardwareMap.get(DcMotor.class, "motor1");
        RBMotor = hardwareMap.get(DcMotor.class, "motor2");
        LBMotor = hardwareMap.get(DcMotor.class, "motor3");
        lift = hardwareMap.get(DcMotor.class, "lift");
        clawLeft = hardwareMap.get(Servo.class, "clawRight");
        clawRight = hardwareMap.get(Servo.class, "clawLeft");
        poleLeft = hardwareMap.get(Rev2mDistanceSensor.class, "pole1");
        poleRight = hardwareMap.get(Rev2mDistanceSensor.class, "pole2");
        arm = hardwareMap.get(Servo.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
// left is port 0, right is port 2
        distanceLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        ticksPerRotation = RFMotor.getMotorType().getTicksPerRev();
        initialFR = RFMotor.getCurrentPosition()/ticksPerRotation;
        initialFL = LFMotor.getCurrentPosition()/ticksPerRotation;
        initialBR = RBMotor.getCurrentPosition()/ticksPerRotation;
        initialBL = LBMotor.getCurrentPosition()/ticksPerRotation;
        liftInitial = (lift.getCurrentPosition()/ticksPerRotation);
       /* LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);*/
        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

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
    public void doCV(){
        // initializeHardware();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
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
        //  telemetry.update();
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
      /*  if(redVal > blueVal && redVal > greenVal){
        //    telemetry.addData("Color", "red");
            result = 1;
        }else if(blueVal > redVal && blueVal > greenVal){
            //telemetry.addData("Color", "blue");
            result = 2;
        }else{
          //  telemetry.addData("Color", "green");
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







