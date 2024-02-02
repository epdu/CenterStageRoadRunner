// Blue left setup
// set the distanct from frot of robot to the block of game element
/*  Using the specs from the motor, you would need to find the encoder counts per revolution (of the output shaft).
     Then, you know that corresponds to 360 degrees of wheel rotation, which means the distance travelled is the circumference
      of the wheel (2 * pi * r_wheel). To figure out how many encoder ticks correspond to the distance you wanna go,
      just multiply the distance by the counts / distance you calculated above. Hope that helps!
// 11.87374348
//537 per revolution 11.87374348 inch
*/

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AutonomousCopyALan extends LinearOpMode {
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    DistanceSensor LeftSensor;
    DistanceSensor RightSensor;
    IMU imu;
    double ticksPerRotation;
    double initialFR;
    double initialFL;
    double initialBR;
    double initialBL;
    double positionFR;
    double positionFL;
    double positionBR;
    double positionBL;
    public String updates;
    public int i = 0;
    double targetheading;
    double heading;
    double previousHeading;
    double processedHeading;
    double  distanceInInch;
    double  distanceInInchDouble;
    private double wheelDiameterInInches = 3.77953;  // Adjust this based on your mecanum wheel diameter
    String teamPropLocations = "Left";
    String PurplePixel;
    //    boolean
    String found;
    double redVal;
    double blueVal;
    double greenVal;
    double liftInitial;
    double liftIdealPos;
    double liftIdealPower;
    int result;

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution
    /*
       1280 x 720 pixels Logitech Webcam C270 (1280 x 720 pixels)
       private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
       private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    */
    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1430;  //Logitech C270  Replace with the focal length of the camera in pixels
//    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    //    Blinker control_Hub;
    //   DcMotor lift;
    //    BNO055IMU imu;
    //    Servo clawLeft;
//    Servo clawRight;
    // @Override
    private static final boolean USE_WEBCAM = true;
    private static int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    final double DESIRED_DISTANCE = 2.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

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

        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ticks per revolution
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

//        if (USE_WEBCAM)
//            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur








 /*
            while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
*/
//        AprilTagOmni(); //find the right april tag and approach it
//        droppixelbackdrop();
//        goparking();


        distanceInInch=24;//number in unit of inch
        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward

        if (gamepad1.options) {
            imu.resetYaw();
        }
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
/*
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftReading = LeftSensor.getDistance(DistanceUnit.INCH);
            double rightReading = RightSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            telemetry.update();
        }
*/

/*
set the distanct from frot of robot to the block of game element
Using the specs from the motor, you would need to find the encoder counts per revolution (of the output shaft).
     Then, you know that corresponds to 360 degrees of wheel rotation, which means the distance travelled is the circumference
      of the wheel (2 * pi * r_wheel). To figure out how many encoder ticks correspond to the distance you wanna go,
      just multiply the distance by the counts / distance you calculated above. Hope that helps!
11.87374348 537 per revolution 11.87374348 inch
*/
        distanceInInch=24;//number in unit of inch
        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        if (gamepad1.options) {
            imu.resetYaw();
        }
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
/*
        run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftReading = LeftSensor.getDistance(DistanceUnit.INCH);
            double rightReading = RightSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Left sensor", (double)(Math.round(leftReading * 10) / 10.0));
            telemetry.addData("Right sensor", (double)(Math.round(rightReading * 10) / 10.0));
            telemetry.update();
        }
*/
        //        findteamPropLocationsbyDistanceSensors(); findteamPropLocationsopencv(); pick up pne of them only

//        moveBackward(0.2, 16);
//        findteamPropLocationsopencv();
//        dropPurplePixel();
        waitForStart();
        while (opModeIsActive()) {
//            found="false";
//            telemetry.addData("Find team prop or not", found);
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//            telemetry.update();
//            PurplePixel="NOTDONE";


//            findteamPropLocationsopencv();
//            dropPurplePixel();
            telemetry.addData("teamprop", teamPropLocations);

            if (teamPropLocations == "Left")
            {
                moveBackward(0.5, 10);
//                DESIRED_TAG_ID = 1;
//                lookfortag(DESIRED_TAG_ID);




            } else if (teamPropLocations == "Center") {
//                DESIRED_TAG_ID = 2;
//                lookfortag(DESIRED_TAG_ID);

            } else if (teamPropLocations == "Right") {
//                DESIRED_TAG_ID = 3;
//                lookfortag(DESIRED_TAG_ID);

            }
            telemetry.update();


//            if(found =="true"){
//                telemetry.addData("Find team prop or not", found);
//                telemetry.update();
//                break;}
            // The OpenCV pipeline automatically processes frames and handles detection
        }


        controlHubCam.stopStreaming();

    }

    public void lookfortag(int tag){
        DESIRED_TAG_ID = tag;
        double drive = 0;
        double turn = 0;
        double strafe = 0;

        boolean targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    telemetry.addData("test", "test");
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if (targetFound) {
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = com.qualcomm.robotcore.util.Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = com.qualcomm.robotcore.util.Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        }
        moveRobot(drive, strafe, turn);
        sleep(10);

    }
    public void initOpenCV() {
        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());
        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    public void  findteamPropLocationsbyDistanceSensors(){
        double leftReading = LeftSensor.getDistance(DistanceUnit.INCH);
        double rightReading = RightSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Left", leftReading);
        telemetry.addData("Right", rightReading);
        telemetry.update();
// L=24.68+- 2, Center=30.56+-2
        if(leftReading > 23 && leftReading < 28 && rightReading > 40){
            teamPropLocations="Left";
            telemetry.addData("Left", leftReading);
            telemetry.addData("Right", rightReading);
            telemetry.addData("teamPropLocations", teamPropLocations);
            telemetry.update();
        } else if ( leftReading > 28 && leftReading < 40 && rightReading > 28 && rightReading < 40) {
            teamPropLocations = "Center";
            telemetry.addData("Left", leftReading);
            telemetry.addData("Right", rightReading);
            telemetry.addData("teamPropLocations", teamPropLocations);
            telemetry.update();
        } else if( leftReading > 40 && rightReading > 40) {
            teamPropLocations = "Right";
            telemetry.addData("Left", leftReading);
            telemetry.addData("Right", rightReading);
            telemetry.addData("teamPropLocations", teamPropLocations);
            telemetry.update();
        }
    }
    public void  findteamPropLocationsopencv(){
//////////////////
        telemetry.addData("cX", cX);
        telemetry.addData("teamPropLocations", teamPropLocations);
        telemetry.update();
// L=24.68+- 2, Center=30.56+-2
///////////////

        if(cX > 0 && cX < 365 ){
            teamPropLocations="Left";
            telemetry.addData("Left", cX);
            telemetry.addData("teamPropLocations", teamPropLocations);
            telemetry.update();
        } else if ( cX > 460 && cX < 820) {
            teamPropLocations = "Center";
            telemetry.addData("Center", cX);
            telemetry.addData("teamPropLocations", teamPropLocations);
            telemetry.update();
        } else if( cX > 915 && cX < 1280) {
            teamPropLocations = "Right";
            telemetry.addData("Right",cX);
            telemetry.addData("teamPropLocations", teamPropLocations);
            telemetry.update();
        }


/////////////

    }
    /////////////////////////////
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
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
//            Scalar lowerBlue = new Scalar(180, 8, 24); // lower bound HSV for blue tested by blue team prop
//            Scalar upperBlue = new Scalar(230, 255, 255);

            Scalar lowerBlue = new Scalar(1, 98, 34); // lower bound HSV for blue tested by blue cone 223 25 31
            Scalar upperBlue = new Scalar(30, 255, 255);

            // Scalar lowerRed = new Scalar(1, 60, 58); // lower bound HSV for red tested by red team prop
            //  Scalar upperRed = new Scalar(10, 255, 255);
  /*
            Scalar lowHSV = new Scalar(123, 25, 31); // lower bound HSV for blue tested by cone 223 25 31
            Scalar highHSV =  new Scalar(143, 255, 255); // higher bound HSV for blue  214, 34, 28       100-140

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
            Core.inRange(hsvFrame, lowerBlue, upperBlue, yellowMask);

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
    public void  dropPurplePixel(){

        if(teamPropLocations == "Left"){
            moveBackward(0.3, 40);
//or select turn to right , gyroTurn
            StrafingRight(0.3, 12);
            RightTurn(0.3,14.5);
            moveBackward(0.3, 20);
            StrafingRight(0.3, 12);
            StrafingLeft(0.3, 18);
            moveForward(0.3, 20);
            gyroTurn(0.3,  150);
//            gyroTurn(0.3,  120);
//            gyroTurn(0.3,  -120);
//           RightTurn(0.3,12);
//           LeftTurn(0.3,12);
            //drop pixel

            //           StrafingLeft(0.3, 12);
//            gyroTurn(0.2, - 90);
//            absoluteHeading( 0.2,  -90);
            found="true";
        } else if ( teamPropLocations == "Right") {
            moveBackward(0.3, 28);

// select turn to left
//            StrafingLeft(0.2, 20);

            gyroTurn(0.2,  90);
//            absoluteHeading( 0.2,  90);
            //drop pixel
            found="true";
        } else if ( teamPropLocations == "Center") {
            moveBackward(0.3, 46);
            absoluteHeading( 0.2,  90);
            //drop pixel
            found="true";
        }
//        checkTeamPropColors();
//        lineUPteamProp();
    }
    public void  checkTeamPropColors(){ }
    public void  lineUPteamProp(){ }
    //////////////////
    public void gyroTurn(double power, double degrees){ //Fright is negative
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
    public void moveForward(double power, double distanceInInch) {

        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setTargetPosition((int) -distanceInInchDouble);
        RBMotor.setTargetPosition((int) -distanceInInchDouble);
        LFMotor.setTargetPosition((int) -distanceInInchDouble);
        LBMotor.setTargetPosition((int) -distanceInInchDouble);
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
    public void moveBackward(double power, double distanceInInch) {
        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setTargetPosition((int) distanceInInchDouble);
        RBMotor.setTargetPosition((int) distanceInInchDouble);
        LFMotor.setTargetPosition((int) distanceInInchDouble);
        LBMotor.setTargetPosition((int) distanceInInchDouble);
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

    public void RightTurn(double power, double distanceInInch) {
        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setTargetPosition((int) -distanceInInchDouble);
        RBMotor.setTargetPosition((int) -distanceInInchDouble);
        LFMotor.setTargetPosition((int) +distanceInInchDouble);
        LBMotor.setTargetPosition((int) +distanceInInchDouble);
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
    public void LeftTurn(double power, double distanceInInch) {
        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setTargetPosition((int) +distanceInInchDouble);
        RBMotor.setTargetPosition((int) +distanceInInchDouble);
        LFMotor.setTargetPosition((int) -distanceInInchDouble);
        LBMotor.setTargetPosition((int) -distanceInInchDouble);
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


    public void StrafingLeft(double power, double distanceInInch) {
        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setTargetPosition((int) +distanceInInchDouble);
        LBMotor.setTargetPosition((int) -distanceInInchDouble);
        RFMotor.setTargetPosition((int) -distanceInInchDouble);
        RBMotor.setTargetPosition((int) +distanceInInchDouble);
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

    public void StrafingRight(double power, double distanceInInch) {
        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setTargetPosition((int) -distanceInInchDouble);
        LBMotor.setTargetPosition((int) +distanceInInchDouble);
        RFMotor.setTargetPosition((int) +distanceInInchDouble);
        RBMotor.setTargetPosition((int) -distanceInInchDouble);
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

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
//        if (USE_WEBCAM) {
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
//        } else {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .addProcessor(aprilTag)
//                    .build();
//        }
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.

        double leftFrontPower    =  -x +y +yaw;
        double rightFrontPower   =  -x -y -yaw;
        double leftBackPower     =  -x -y +yaw;
        double rightBackPower    =  -x +y -yaw;
//        double fl = y + x + rx;
//        double fr = y - x - rx;
//        double bl = y - x + rx;
//        double br = y + x - rx;
// good for our team

//original set up
//        double leftFrontPower    =  x -y -yaw;
//        double rightFrontPower   =  x +y +yaw;
//        double leftBackPower     =  x +y -yaw;
//        double rightBackPower    =  x -y +yaw;
//original set up


        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        LFMotor.setPower(leftFrontPower);
        RFMotor.setPower(rightFrontPower);
        LBMotor.setPower(leftBackPower);
        RBMotor.setPower(rightBackPower);
    }



}


