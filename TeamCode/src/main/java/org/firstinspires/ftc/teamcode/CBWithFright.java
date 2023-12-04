
/*
package org.firstinspires.ftc.teamcode;

import java.util.Date;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import java.util.Locale;
@Disabled
public class CBWithFreight extends DrivingFunctionsAutonomous{
    private int liftLevel;
    public void doRun(int reverse){ //1 for red, -1 for blue
        initializeHardware();
        waitForStart();
        claw.setPosition(1);
        sleep(400);
        telemetry.addData("Left sensor:", distance2.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right sensor:", distance1.getDistance(DistanceUnit.INCH));
        telemetry.update();
/*if(reverse == 1){
if(distance2.getDistance(DistanceUnit.INCH)<22 &&
distance2.getDistance(DistanceUnit.INCH) > 10){
liftLevel = 1;
}else if(distance1.getDistance(DistanceUnit.INCH)<22){
liftLevel = 2;
}else{
liftLevel = 3;
}
}else{
if(distance1.getDistance(DistanceUnit.INCH)<22){
liftLevel = 3;
}else if(distance2.getDistance(DistanceUnit.INCH)<22){
liftLevel = 2;
}else{
liftLevel = 1;
}
}
setLiftLevel(liftLevel);
goStraight(.4,.05);
gyroTurn(.4,35*reverse);
setLiftLevel(liftLevel);
if(liftLevel == 1){
goStraight(.4,.4);
}else if(liftLevel == 2){
goStraight(.4,.28);
}else{
goStraight(.4,.22);
}
sleep(400);
claw.setPosition(.75);
sleep(400);
lift.setPower(0);
if(liftLevel == 1){
goBackward(.4,.29);
}else if(liftLevel == 2){
goBackward(.4,.2);
}else{
goBackward(.4,.19);
}
if(liftLevel!=3){
gyroTurn(.4,-35*reverse);
}else{
gyroTurn(.4,-37*reverse);
}
setLiftLevel(0);
if(reverse==1){
strafeLeft(.35,.56,3);
}else{
strafeRight(.35,.56,3);
}
goBackward(.25,.06);
duckSpinner.setPower(-.5*reverse);
sleep(3500);
duckSpinner.setPower(0);*/

/*
        if(reverse == 1){
            if(distance2.getDistance(DistanceUnit.INCH)<22){
                liftLevel = 1;
            }else if(distance1.getDistance(DistanceUnit.INCH)<22){
                liftLevel = 2;
            }else{
                liftLevel = 3;
            }
        }else{
            if(distance1.getDistance(DistanceUnit.INCH)<22){
                liftLevel = 1;
            }else if(distance2.getDistance(DistanceUnit.INCH)<22){
                liftLevel = 2;
            }else{
                liftLevel = 3;
            }
        }
        setLiftLevel(liftLevel);
        goStraight(.25,.05);
        gyroTurn(.25,35*reverse);
        setLiftLevel(liftLevel);
        if(liftLevel == 1){
            goStraight(.25,.41);
        }else if(liftLevel == 2){
            goStraight(.25,.33);
        }else{
            goStraight(.25,.27);
        }
        sleep(400);
        claw.setPosition(.75);
        sleep(400);
        lift.setPower(0);
        if(liftLevel == 1){
            goBackward(.25,.3);
        }else if(liftLevel == 2){
            goBackward(.25,.24);
        }else{
            goBackward(.25,.16);
        }
        gyroTurn(.25,-35*reverse);
        setLiftLevel(0);
        if(reverse==1){
            strafeLeft(.35,.56,3);
        }else{
            strafeRight(.35,.56,3);
        }
        goBackward(.25,.06);
        duckSpinner.setPower(-.5*reverse);
        sleep(3500);
        duckSpinner.setPower(0);
        setLiftLevel(1);
        if(liftLevel == 1){
            if(reverse == 1){
                strafeRight(.6,1.27,5);
            }else{
                strafeLeft(.6,1.27,5);
            }
            setLiftLevel(0);
            motorFR.setPower(.25);
            motorBR.setPower(.25);
            motorFL.setPower(.25);
            motorBL.setPower(.25);
//THING HERE
            double startPos = (motorFR.getCurrentPosition()/ticksPerRotation)-initialFR;
            double nowPos = (motorFR.getCurrentPosition()/ticksPerRotation)-initialFR;
            boolean tryDuck = true;
            while(clawDistance.getDistance(DistanceUnit.INCH)>4.5 && opModeIsActive() && nowPos-startPos < .15){
                sleep(1);
                nowPos = (motorFR.getCurrentPosition()/ticksPerRotation)-initialFR;
// if(nowPos-startPos <.3){
// tryDuck = false;
// }
            }
            stopMotors();
// if(tryDuck){
            sleep(100);
            claw.setPosition(1);
            sleep(100);
            setLiftLevel(3);
            goBackward(.25,.07);
            gyroTurn(.25,-30*reverse);
            sleep(250);
            claw.setPosition(.75);
            sleep(250);
            gyroTurn(.25,-50*reverse);
            setLiftLevel(1);
            sleep(250);
            if(reverse == 1){
                strafeRight(.5,.15,2);
            }else{
                strafeLeft(.5,.15,2);
            }
            goBackward(1,.75);
            sleep(500);
            setLiftLevel(0);
/*}
else{
goBackward(.5,.14);
gyroTurn(.25,-88);
setLiftLevel(1);
goBackward(1,1);
setLiftLevel(0);
}*/

/*
        }else if(liftLevel == 2){
            if(reverse == 1){
                strafeRight(.6,1.41,5);
            }else{
                strafeLeft(.6,1.41,5);
            }
            setLiftLevel(0);
            motorFR.setPower(.25);
            motorBR.setPower(.25);
            motorFL.setPower(.25);
            motorBL.setPower(.25);
            double startPos = (motorFR.getCurrentPosition()/ticksPerRotation)-initialFR;
            double nowPos = (motorFR.getCurrentPosition()/ticksPerRotation)-initialFR;
            boolean tryDuck = true;
            while(clawDistance.getDistance(DistanceUnit.INCH)>4.5 && opModeIsActive() && nowPos-startPos < .15){
                sleep(1);
                nowPos = (motorFR.getCurrentPosition()/ticksPerRotation)-initialFR;
/* if(nowPos-startPos < .3){
tryDuck = false;
}*/


/*
            }
            stopMotors();
//if(tryDuck){
            sleep(100);
            claw.setPosition(1);
            sleep(100);
            setLiftLevel(3);
            gyroTurn(.25,-45*reverse);
            claw.setPosition(.75);
            sleep(500);
            gyroTurn(.5,-42*reverse);
            if(reverse == 1){
                strafeLeft(.5,.15,5);
            }else{
                strafeRight(.5,.15,5);
            }
            setLiftLevel(2);
            sleep(500);
            goBackward(1,.7);
            claw.setPosition(1);
            sleep(500);
            setLiftLevel(0);
            if(reverse == 1){
                strafeLeft(.75,.1,5);
            }else{
                strafeRight(.75,.1,5);
            }
/* }else{
gyroTurn(.25,-88);
strafeLeft(.5,.2,5);
setLiftLevel(1);
goBackward(1,.75);
setLiftLevel(0);
}*/

/*
        }else if(liftLevel == 3){
            if(reverse == 1){
                strafeRight(.6,1.3,5);
            }else{
                strafeLeft(.6,1.3,5);
            }
            setLiftLevel(0);
            gyroTurn(.5,86*reverse);
            if(reverse == 1){
                strafeLeft(.6,.37,3);
            }else{
                strafeRight(.6,.37,3);
            }
//goStraight(.25,.15);
            motorFR.setPower(.25);
            motorFL.setPower(.25);
            motorBR.setPower(.25);
            motorBL.setPower(.25);
            double posthing = motorFR.getCurrentPosition()/ticksPerRotation;
            double currentpos = motorFR.getCurrentPosition()/ticksPerRotation;
            long starttime = System.currentTimeMillis();
            long currenttime = System.currentTimeMillis();
            while(currentpos-posthing<.15 && currenttime-starttime<2000 && opModeIsActive()){ currenttime = System.currentTimeMillis();
                currentpos = motorFR.getCurrentPosition()/ticksPerRotation;
                sleep(1);
            }
            stopMotors();
// sleep(100);
            claw.setPosition(1);
            sleep(500);
            goBackward(.25,.1);
            gyroTurn(.5,-140*reverse);
            claw.setPosition(1);
            sleep(500);
            setLiftLevel(1);
            goStraight(.5,.035);
            claw.setPosition(.75);
            sleep(100);
            goBackward(.5,.03);
            gyroTurn(.25,-30*reverse);
            if(reverse == 1){
                strafeLeft(.5,.05,1);
            }else{
                strafeRight(.5,.05,1);
            }
            goBackward(1,.75);
            sleep(500);
            claw.setPosition(1);
            setLiftLevel(0);
        }
    }
}


*/