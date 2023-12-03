package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class AutonomousCode extends LinearOpMode {

    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    Blinker control_Hub;
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

// set the distanct from frot of robot to the block of game element
     /*Using the specs from the motor, you would need to find the encoder counts per revolution (of the output shaft).
     Then, you know that corresponds to 360 degrees of wheel rotation, which means the distance travelled is the circumference
      of the wheel (2 * pi * r_wheel). To figure out how many encoder ticks correspond to the distance you wanna go,
      just multiply the distance by the counts / distance you calculated above. Hope that helps!
        */

        findPixel();
//        droppixel();
        moveBackward(0.2, 10);
        sleep(2000);
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

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            double rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            telemetry.update();
        }

    }

    private void goparking() {
    }

    private void AprilTagOmni() {
    }

    private void droppixelbackdrop() {
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
    public void droppixel(){}
    //
    public void findPixel(){
        double power = .25;
        double multiplier;
        targetheading = getHeading();
        double leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
        double rightReading = distanceRight.getDistance(DistanceUnit.INCH);
        telemetry.addData("Left", leftReading);
        telemetry.addData("Right", rightReading);
        telemetry.update();
        if(leftReading > 32 && rightReading > 32 || leftReading < 10 || rightReading < 10){
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
            leftReading = distanceLeft.getDistance(DistanceUnit.INCH);
            rightReading = distanceRight.getDistance(DistanceUnit.INCH);
            i++;
        } //end of the left-right error loop
        stopMotors();
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
    public void stopMotors() {
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);
    }
}


