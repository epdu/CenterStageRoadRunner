        /* Copyright (c) 2017 FIRST. All rights reserved.
         *
         * Redistribution and use in source and binary forms, with or without modification,
         * are permitted (subject to the limitations in the disclaimer below) provided that
         * the following conditions are met:
         *
         * Redistributions of source code must retain the above copyright notice, this list
         * of conditions and the following disclaimer.
         *
         * Redistributions in binary form must reproduce the above copyright notice, this
         * list of conditions and the following disclaimer in the documentation and/or
         * other materials provided with the distribution.
         *
         * Neither the name of FIRST nor the names of its contributors may be used to endorse or
         * promote products derived from this software without specific prior written permission.
         *
         * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
         * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
         * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
         * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
         * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
         * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
         * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
         * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
         * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
         * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
         * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
         */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is powerpuffs`s robot from Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *    DcMotor RFMotor;
 *    DcMotor LFMotor;
 *    DcMotor RBMotor;
 *    DcMotor LBMotor;
 * RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
 * LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
 * RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
 * LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 *
 *
 */
//Good version 02082024
public class HardwarePowerpuffs
{
    /* local OpMode members. */
    HardwareMap hwMap =  null;
    /* Public OpMode members. */

    public DcMotor RFMotor;
    public DcMotor LFMotor;
    public DcMotor RBMotor;
    public DcMotor LBMotor;
    public DcMotor liftMotorL;
    public DcMotor liftMotorR;
    public Servo ClawR;
    public Servo ClawL;
    public Servo Wrist;
    public Servo ArmR;
    public Servo ArmL;
    public Servo Drone;

   IMU imu;
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LFMotor   = hwMap.get(DcMotor.class, "LFMotor");//02022024 control hub port 0
        RFMotor  = hwMap.get(DcMotor.class, "RFMotor"); //02022024 control hub port 1
        LBMotor   = hwMap.get(DcMotor.class, "LBMotor");//02022024 control hub port 2
        RBMotor  = hwMap.get(DcMotor.class, "RBMotor");//02022024 control hub port 3

//        LFMotor.setDirection(DcMotor.Direction.FORWARD);
//        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorL = hwMap.get(DcMotor.class, "liftMotorL");//02022024 control hub? port 1
        liftMotorR = hwMap.get(DcMotor.class, "liftMotorR");//02022024 control hub? port 0

        int positionL = liftMotorL.getCurrentPosition();
        int positionR = liftMotorR.getCurrentPosition();
        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotorL.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        liftMotorR.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to zero power
        setAllPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Drone = hardwareMap.get(Servo.class, "Drone");
//        Drone.setPosition(0);

//        Wrist = hardwareMap.get(Servo.class, "wrist");
//        Wrist.setDirection(Servo.Direction.REVERSE);
//        Wrist.setPosition(1);
//
//        ClawR = hardwareMap.get(Servo.class, "ClawR");
//        ClawL = hardwareMap.get(Servo.class, "ClawL");
//        ClawR.setPosition(0.71);
//        ClawL.setPosition(0.505);
//        ClawL.setDirection(Servo.Direction.REVERSE);
//
//        ArmL = hardwareMap.get(Servo.class, "ArmL");
//        ArmR = hardwareMap.get(Servo.class, "ArmR");
//        ArmL.setDirection(Servo.Direction.REVERSE);
/*
         //Wrist = hardwareMap.get(Servo.class, "wrist");//control hub port 5
        Wrist.setPosition(0.34);
        //drone expansion hub port 5
       //ClawR = hardwareMap.get(Servo.class, "ClawR");//control hub port 2
        //ClawL = hardwareMap.get(Servo.class, "ClawL");//control hub port 3
        ClawR.setPosition(0.78);
        ClawL.setPosition(0.018);
        ClawL.setDirection(Servo.Direction.REVERSE);

        //ArmL = hardwareMap.get(Servo.class, "ArmL");//control hub port 1
        //ArmR = hardwareMap.get(Servo.class, "ArmR");//control hub port 0

*/


        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        imu.initialize(parameters);
//        parameters.angleUnit           = IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.imuOrientationOnRobot = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

    }
    //Set power to all motors

    public void setMotorPower(double lF, double rF, double lB, double rB){
        LFMotor.setPower(lF);
        LBMotor.setPower(lB);
        RBMotor.setPower(rB);
        RFMotor.setPower(rF);
    }
    public void setAllPower(double p){
        setMotorPower(p,p,p,p);
    }
}

