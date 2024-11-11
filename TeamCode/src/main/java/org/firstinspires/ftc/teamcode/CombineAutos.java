package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class CombineAutos extends LinearOpMode {
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution
    /*
   private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
       private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

   1280 x 720 pixels
   Logitech Webcam C270 (1280 x 720 pixels)
    */
    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1430;  //Logitech C270  Replace with the focal length of the camera in pixels
//    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;


    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();
    private double targetAngle;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;

    @Override
    public void runOpMode() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new opencv.YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();

//        final int STAGE = 1;
////        final int STAGE = 2;
//        if (STAGE == 1) {
//            turn(90);
//            sleep(3000);
//            turnTo(-90);
//        } else if (STAGE == 2) {
//            turnPID(90);
//        }
    }


    static class YellowBlobDetectionPipeline extends OpenCvPipeline {
        //change yellow to red and/or blue to make it make more sense
        private double width;
        private double cX;
        private double cY;
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;

        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(1, 98, 34); // lower bound HSV for blue tested by blue cone 223 25 31
            Scalar upperYellow = new Scalar(30, 255, 255);

  /*
            Scalar lowHSV = new Scalar(123, 25, 31); // lower bound HSV for blue tested by cone 223 25 31
            Scalar highHSV =  new Scalar(143, 255, 255); // higher bound HSV for blue
214, 34, 28       100-140

        Scalar lowHSV = new Scalar(1, 98, 34); // lower bound HSV for red tested by cone 10, 98, 34
        Scalar highHSV =  new Scalar(20, 255, 255); // higher bound HSV for red

        Scalar lowHSV = new Scalar(200, 32, 49); // lower bound HSV for green tested by pixel 111, 32, 49
        Scalar highHSV =  new Scalar(121, 255, 255); // higher bound HSV for green

        Scalar lowHSV = new Scalar(x, 19, 76); // lower bound HSV for purple pixel 284, 13, 55
        Scalar highHSV = new Scalar(x, 255, 255);  // higher bound HSV for

        Scalar lowHSV = new Scalar(0, 0, 100); // lower bound HSV for white pixel 0, 0, 100
        Scalar highHSV = new Scalar(10, 255, 255);  // higher bound HSV for
                Scalar lowHSV = new Scalar(33, 99, 65); // lower bound HSV for yellow pixel 43, 99, 65
        Scalar highHSV = new Scalar(53, 255, 255);  // higher bound HSV for

        Now you take [H-10, 100,100] and [H+10, 255, 255] as the lower bound and upper bound respectively.
        Apart from this method, you can use any image editing tools like GIMP or any online converters to find these values,
        but don't forget to adjust the HSV ranges.

        Scalar lowHSV = new Scalar(100, 100, 100);  // lower bound HSV for red
        Scalar highHSV =  new Scalar(180, 255, 255);  // higher bound HSV for red
        Scalar lowHSV = new Scalar(95, 110, 50); // lower bound HSV for blue
        Scalar highHSV =  new Scalar(150, 245, 255); // higher bound HSV for blue

        Scalar lowHSV = new Scalar(89, 67, 61); // lower bound HSV for green
        Scalar highHSV = new Scalar(83, 43, 83); // higher bound HSV for green
        Scalar lowHSV = new Scalar(23, 50, 70); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255);  // higher bound HSV for yellow

        Scalar lowHSV = new Scalar(100, 100, 100);  // lower bound HSV for red
        Scalar highHSV =  new Scalar(180, 255, 255);  // higher bound HSV for red

        Scalar lowHSV = new Scalar(95, 110, 50); // lower bound HSV for blue
        Scalar highHSV =  new Scalar(150, 245, 255); // higher bound HSV for blue

        Scalar lowHSV = new Scalar(89, 67, 61); // lower bound HSV for green
        Scalar highHSV = new Scalar(83, 43, 83); // higher bound HSV for green
*/

            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        // Get current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.setAllPower(0);
    }

    public void turnTo(double degrees){

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        System.out.println(orientation.firstAngle);
        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn(error);
    }

    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
    }
    public CombineAutos(double target, double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
        targetAngle = target;
    }

    public double update(double currentAngle) {
        // TODO: make sure angles are within bounds and are in same format (e.g., 0 <= | angle | <= 180)
        //   and ensure direction is correct

        // P
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) {
            error -= 360;
        }

        // I
        accumulatedError *= Math.signum(error);
        accumulatedError += error;
        if (Math.abs(error) < 2) {
            accumulatedError = 0;
        }

        // D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();

        double motorPower = 0.1 * Math.signum(error)
                + 0.9 * Math.tanh(kP * error + kI * accumulatedError - kD * slope);
        return motorPower;
    }

    public double getLastSlope() {
        return lastSlope;
    }


}

