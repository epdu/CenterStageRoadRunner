package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class CHATGPTautonomous extends LinearOpMode {

    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;

    private double countsPerInch;  // Adjust this based on your robot's calibration
    private double wheelDiameterInInches = 3.77953;  // Adjust this based on your mecanum wheel diameter

    @Override
    public void runOpMode() {

        // Initialize motors and set directions
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");


        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set up encoders
        resetEncoders();

        // Calculate counts per inch
        countsPerInch = calculateCountsPerInch();

        waitForStart();

        // Example: Move forward 24 inches
        moveForwardInches(24);

        // Additional autonomous movements can be added here

        stopMotors();
    }

    private void resetEncoders() {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double calculateCountsPerInch() {
        double wheelCircumference = Math.PI * wheelDiameterInInches;
        return (LFMotor.getMotorType().getTicksPerRev() / wheelCircumference);
    }

    private void moveForwardInches(double inches) {
        int targetPosition = (int) (inches * countsPerInch);

        setTargetPosition(targetPosition, targetPosition, targetPosition, targetPosition);

        setMotorPower(0.5);

        while (opModeIsActive() && LFMotor.isBusy() && RFMotor.isBusy()
                && LBMotor.isBusy() && RBMotor.isBusy()) {
            // Wait for the motors to reach the target position
            // Additional operations or telemetry can be added here
        }

        stopMotors();
    }

    private void setTargetPosition(int frontLeft, int frontRight, int rearLeft, int rearRight) {
        LFMotor.setTargetPosition(frontLeft);
        RFMotor.setTargetPosition(frontRight);
        LBMotor.setTargetPosition(rearLeft);
        RBMotor.setTargetPosition(rearRight);

    }

    private void setMotorPower(double power) {
        LFMotor.setPower(power);
        RFMotor.setPower(power);
        LBMotor.setPower(power);
        RBMotor.setPower(power);
    }

    private void stopMotors() {
        setMotorPower(0);
    }
}

