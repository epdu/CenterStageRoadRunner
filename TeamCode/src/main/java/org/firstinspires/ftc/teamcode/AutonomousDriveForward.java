package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class AutonomousDriveForward extends LinearOpMode{
    DcMotor frontRight;
    DcMotor rearRight;
    DcMotor rearLeft;
    DcMotor frontLeft;
    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.get(DcMotor.class, "RFMotor");
        frontLeft = hardwareMap.get(DcMotor.class, "LFMotor");
        rearRight= hardwareMap.get(DcMotor.class, "RBMotor");
        rearLeft = hardwareMap.get(DcMotor.class, "LBMotor");

//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

    // Wait for the game to start (driver presses PLAY)

    // Autonomous code goes here
    // Example: Move forward for 2 seconds
    //forward
    mecanumDrive(-0.2, 0, 0);
    sleep(2000); // Adjust sleep time based on how long you want the robot to move
    //strafing left
    mecanumDrive(0, 0.2, 0);
    sleep(2000); // Adjust sleep time based on how long you want the robot to move
    // turn to left
    mecanumDrive( 0, 0, 0.2);
    sleep(2000);
    //backward
     mecanumDrive( 0.2, 0, 0);
     sleep(2000);
     //strafing right
     mecanumDrive( 0, -0.2, 0);
     sleep(2000);
     // turn to right
     mecanumDrive( 0, 0, -0.2);
     sleep(2000);

     stopRobot();


}

    // Mecanum drive method
    private void mecanumDrive(double forward, double strafe, double rotate) {
        frontLeft.setPower(forward + strafe + rotate);
        rearLeft.setPower(forward - strafe + rotate);
        frontRight.setPower(forward - strafe - rotate);
        rearRight.setPower(forward + strafe - rotate);


    }

    // Stop the robot
    private void stopRobot() {
        mecanumDrive(0, 0, 0);
    }
}


