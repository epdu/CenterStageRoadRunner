//// Blue left setup
//// set the distanct from frot of robot to the block of game element
///*  Using the specs from the motor, you would need to find the encoder counts per revolution (of the output shaft).
//     Then, you know that corresponds to 360 degrees of wheel rotation, which means the distance travelled is the circumference
//      of the wheel (2 * pi * r_wheel). To figure out how many encoder ticks correspond to the distance you wanna go,
//      just multiply the distance by the counts / distance you calculated above. Hope that helps!
//// 11.87374348
////537 per revolution 11.87374348 inch
//*/
//
//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.openftc.easyopencv.OpenCvCamera;
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name = "Autonomous of alan")
//public class AutonomousCopyALan extends LinearOpMode {
//    DcMotor RFMotor;
//    DcMotor LFMotor;
//    DcMotor RBMotor;
//    DcMotor LBMotor;
//    DcMotor liftMotorL;
//    DcMotor liftMotorR;
//    Servo ClawR;
//    Servo ClawL;
//    Servo Wrist;
//    Servo ArmR;
//    Servo ArmL;
//    Servo Drone;
//    public float speedMultiplier=0.5f;
//    public float speedLimiter =0.5f;
//    DistanceSensor LeftSensor;
//    DistanceSensor RightSensor;
//    IMU imu;
//    double ticksPerRotation;
//    double initialFR;
//    double initialFL;
//    double initialBR;
//    double initialBL;
//    double positionFR;
//    double positionFL;
//    double positionBR;
//    double positionBL;
//    public String updates;
//    public int i = 0;
//    double targetheading;
//    double heading;
//    double previousHeading;
//    double processedHeading;
//    double  distanceInInch;
//    double  distanceInInchDouble;
//    double  distanceRFMotor;
//    double  distanceRBMotor;
//    double  distanceLFMotor;
//    double  distanceLBMotor;
//
//    private double wheelDiameterInInches = 3.77953;  // Adjust this based on your mecanum wheel diameter
//    String teamPropLocations;  //= "Left"
//    String PurplePixel;
//    //    boolean
//    String found;
//    double redVal;
//    double blueVal;
//    double greenVal;
//    double liftInitial;
//    double liftIdealPos;
//    double liftIdealPower;
//    int result;
//    private OpenCvCamera openCvCamera = null;
//    double cX = 0;
//    double cY = 0;
//    double width = 0;
//
//    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
//    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution
//    /*
//       1280 x 720 pixels Logitech Webcam C270 (1280 x 720 pixels)
//       private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
//       private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
//    */
//    // Calculate the distance using the formula
//    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
//    public static final double focalLength = 1430;  //Logitech C270  Replace with the focal length of the camera in pixels
////    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
//
//    private static final boolean USE_WEBCAM = true;
//    private static int DESIRED_TAG_ID = -1;
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//    private OpenCvVisionProcessor redTeamPropOpenCv;
//    private OpenCvVisionProcessor blueTeamPropOpenCv;
//    private AprilTagDetection desiredTag = null;
//    final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)
//
//    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
//    //  applied to the drive motors to correct the error.
//    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
//    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
//    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Retrieve the IMU from the hardware map
//        imu = hardwareMap.get(IMU.class, "imu");
////        LeftSensor = hardwareMap.get(DistanceSensor.class, "DistanceLeft");
////        RightSensor = hardwareMap.get(DistanceSensor.class, "DistanceRight");
//        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");//02022024 control hub port 1
//        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor"); //02022024 control hub port 0
//        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");//02022024 control hub port 3
//        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");//02022024 control hub port 2
//
//        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // ticks per revolution
//        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        Wrist = hardwareMap.get(Servo.class, "wrist");
//        Wrist.setPosition(0.34);
//
//        ClawR = hardwareMap.get(Servo.class, "ClawR");
//        ClawL = hardwareMap.get(Servo.class, "ClawL");
//        ClawR.setPosition(0.78);
//        ClawL.setPosition(0.018);
//        ClawL.setDirection(Servo.Direction.REVERSE);
//
//        ArmL = hardwareMap.get(Servo.class, "ArmL");
//        ArmR = hardwareMap.get(Servo.class, "ArmR");
//
//        ArmL.setPosition(0.5);
//        ArmR.setPosition(0.5);
//
////        initOpenCV();
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
//
//        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
//        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
//        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
//        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
//
//        // Initialize the Apriltag Detection process
//        initVisionPortal() ;
//
//
//        distanceInInch=24;//number in unit of inch
//        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
//        // This button choice was made so that it is hard to hit on accident,
//        // it can be freely changed based on preference.
//        // The equivalent button is start on Xbox-style controllers.
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//
//        if (gamepad1.options) {
//            imu.resetYaw();
//        }
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//
///*
//set the distance from front of robot to the block of game element
//Using the specs from the motor, you would need to find the encoder counts per revolution (of the output shaft).
//     Then, you know that corresponds to 360 degrees of wheel rotation, which means the distance travelled is the circumference
//      of the wheel (2 * pi * r_wheel). To figure out how many encoder ticks correspond to the distance you wanna go,
//      just multiply the distance by the counts / distance you calculated above. Hope that helps!
//11.87374348 537 per revolution 11.87374348 inch
//*/
//
//
//        waitForStart();
//
//
//        while (opModeIsActive()) {
//            // TODO: Need to do red or blue according to alliance color.
//            Point teamPropCentroid = redTeamPropOpenCv.getTeamPropCentroid();
//            cX = teamPropCentroid.x;
//            cY = teamPropCentroid.y;
//            found= cX != 0.0 || cY != 0.0 ? "true" : "false";
//            telemetry.addData("Find team prop or not", found);
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//            telemetry.update();
//            PurplePixel="NOTDONE";
////add class or method here
//
//            findteamPropLocations();
//            dropPurplePixel();
//            aprilTagOmni();
//            dropYellowPixel();
//            autoParking();
//        }
//
//        controlHubCam.stopStreaming();
//    }
//
//    public void lookfortag(int tag){
//        DESIRED_TAG_ID = tag;
//        double drive = 0;
//        double turn = 0;
//        double strafe = 0;
//
//        boolean targetFound = false;
//        desiredTag  = null;
//
//        // Step through the list of detected tags and look for a matching tag
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        for (AprilTagDetection detection : currentDetections) {
//            // Look to see if we have size info on this tag.
//            if (detection.metadata != null) {
//                //  Check to see if we want to track towards this tag.
//                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                    // Yes, we want to use this tag.
//                    targetFound = true;
//                    desiredTag = detection;
//                    telemetry.addData("test", "test");
//                    break;  // don't look any further.
//                } else {
//                    // This tag is in the library, but we do not want to track it right now.
//                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                }
//            } else {
//                // This tag is NOT in the library, so we don't have enough information to track to it.
//                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//            }
//        }
//
//        if (targetFound) {
//            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
//            double headingError = desiredTag.ftcPose.bearing;
//            double yawError = desiredTag.ftcPose.yaw;
//            // Use the speed and turn "gains" to calculate how we want the robot to move.
//            drive = com.qualcomm.robotcore.util.Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            turn = com.qualcomm.robotcore.util.Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//        }
//        moveRobot(drive, strafe, turn);
//        sleep(10);
//
//    }
//
//
//    public String  findteamPropLocations(){
////////////////////
//        telemetry.addData("cX", cX);
//        telemetry.addData("teamPropLocations", teamPropLocations);
//        telemetry.update();
//
//        if(cX > 0 && cX < 365 ){
//            teamPropLocations="Left";
//            telemetry.addData("Left", cX);
//            telemetry.addData("teamPropLocations", teamPropLocations);
//            telemetry.update();
//        } else if ( cX > 460 && cX < 820) {
//            teamPropLocations = "Center";
//            telemetry.addData("Center", cX);
//            telemetry.addData("teamPropLocations", teamPropLocations);
//            telemetry.update();
//        } else if( cX > 915 && cX < 1280) {
//            teamPropLocations = "Right";
//            telemetry.addData("Right",cX);
//            telemetry.addData("teamPropLocations", teamPropLocations);
//            telemetry.update();
//        }
//        return teamPropLocations;
//    }
//
//    private static double getDistance(double width){
//        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
//        return distance;
//    }
//    public void  dropPurplePixel(){
//
//        if(teamPropLocations == "Left"){
//            moveBackward(0.3, 40);  // set robot backward for camera to see the team prop,move 40 to approcah the team prop
//            strafingRight(0.3, 12); //line up the claw of the side holding purple pixel
//            turnRight(0.3,14.5); //dropped the pixel, and move to backdrop
//            moveBackward(0.3, 16); //approaching backdrop
//            strafingRight(0.3, 22);//move parallel the april tags at the bottom of backdrop in order to locate them
//            moveBackward(0.3, 5);
//            moveForward(0.3, 20);
//
//            found="true";
//        } else if ( teamPropLocations == "Right") {
//            moveBackward(0.3, 28);
//
//            found="true";
//        } else if ( teamPropLocations == "Center") {
//            moveBackward(0.3, 46);
//
//            found="true";
//        }
//
//    }
//
//    public void  dropYellowPixel(){
//        // move arms and then open claw
//    }
//    public void  autoParking(){
//        moveForward(0.3, 5);
//        strafingRight(0.3, 12);
//        moveBackward(0.3, 40);  // set robot backward for camera to see the team prop,move 40 to approcah the team prop
//        strafingRight(0.3, 12); //line up the claw of the side holding purple pixel
//        turnRight(0.3,14.5);//dropped the pixel, and move to backdrop
//        moveBackward(0.3, 16); //approaching backdrop
//        strafingRight(0.3, 22);//move parallel the april tags at the bottom of backdrop in order to locate them
//        moveBackward(0.3, 5);
//
//    }
//    //work here
//
//    public void  aprilTagOmni(){
//
//
//        if (teamPropLocations == "Left")
//        {
//            moveBackward(0.5, 1);
//            DESIRED_TAG_ID = 1;
//            lookfortag(DESIRED_TAG_ID);
//
//        } else if (teamPropLocations == "Center") {
////                DESIRED_TAG_ID = 2;
////                lookfortag(DESIRED_TAG_ID);
//
//        } else if (teamPropLocations == "Right") {
////                DESIRED_TAG_ID = 3;
////                lookfortag(DESIRED_TAG_ID);
//
//        }
//    }
//    public void  checkTeamPropColors(){ }
//    public void  lineUPteamProp(){ }
//    //////////////////
//
//
//
//    public void stopMotors(){
//        RFMotor.setPower(0);
//        LFMotor.setPower(0);
//        RBMotor.setPower(0);
//        LBMotor.setPower(0);
//    }
//
//    private void goparking() {
//    }
//
//    private void AprilTagOmni() {
//    }
//
//    private void droppixelbackdrop() {
//    }
//    public void droppixel(){}
//    //
//    //test function]
//    //
//    //
//    public void moveForward(double power, double distanceInInch) {
//        movement(power, -distanceInInch,-distanceInInch,-distanceInInch,-distanceInInch) ;
//    }
//    public void moveBackward(double power, double distanceInInch) {
//        movement(power, +distanceInInch,+distanceInInch,+distanceInInch,+distanceInInch) ;
//    }
//    public void turnRight(double power, double distanceInInch) {
//        movement(power, +distanceInInch,+distanceInInch,-distanceInInch,-distanceInInch);
//    }
//    public void turnLeft(double power, double distanceInInch) {
//        movement(power, +distanceInInch,-distanceInInch,-distanceInInch,+distanceInInch);
//     }
//    public void strafingRight(double power, double distanceInInch) {
//        movement(power, -distanceInInch,+distanceInInch,+distanceInInch,-distanceInInch);
//    }
//    public void strafingLeft(double power, double distanceInInch) {
//        movement(power, +distanceInInch,-distanceInInch,-distanceInInch,+distanceInInch);
//    }
//
//    public void movement(double power, double distanceRF,double distanceRB,double distanceLF,double distanceLB) {
////input distance in inches, robot will finish movement "move forward backward, shraftleft and right,turn left and right"
//        distanceRFMotor=(double)(distanceRF*537/(Math.PI * wheelDiameterInInches));
//        distanceRBMotor=(double)(distanceRB*537/(Math.PI * wheelDiameterInInches));
//        distanceLFMotor=(double)(distanceLF*537/(Math.PI * wheelDiameterInInches));
//        distanceLBMotor=(double)(distanceLF*537/(Math.PI * wheelDiameterInInches));
//        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RFMotor.setTargetPosition((int) -distanceRFMotor);
//        RBMotor.setTargetPosition((int) -distanceRBMotor);
//        LFMotor.setTargetPosition((int) -distanceLFMotor);
//        LBMotor.setTargetPosition((int) -distanceRBMotor);
//        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RFMotor.setPower(+power);
//        RBMotor.setPower(+power);
//        LFMotor.setPower(+power);
//        LBMotor.setPower(+power);
//        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {}
//        RFMotor.setPower(0);
//        LFMotor.setPower(0);
//        RBMotor.setPower(0);
//        LBMotor.setPower(0);
//    }
//    private void initVisionPortal() {
//
//        aprilTag = new AprilTagProcessor.Builder().build();
//        redTeamPropOpenCv= new OpenCvVisionProcessor("Red", new Scalar(1, 98, 34), new Scalar(30, 255, 255) );
//        blueTeamPropOpenCv= new OpenCvVisionProcessor("Blue", new Scalar(180, 8, 24), new Scalar(230, 255, 255));
//
///*
//        redTeamPropOpenCv= new OpenCvVisionProcessor("Red", new Scalar(0, 10, 120), new Scalar(100, 255, 255) );
//        redTeamPropOpenCv= new OpenCvVisionProcessor("Red", new Scalar(125, 120, 50), new Scalar(190, 255, 255) );
//        blueTeamPropOpenCv= new OpenCvVisionProcessor("Blue", new Scalar(160, 200,120), new Scalar(100, 255, 255) );
//        blueTeamPropOpenCv= new OpenCvVisionProcessor("Blue", new Scalar(130, 120, 50), new Scalar(130, 255, 255) );
//*/
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(2);
//
//        // Create the vision portal by using a builder.
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessors(aprilTag)
//                .addProcessor(redTeamPropOpenCv)
//                .addProcessor(blueTeamPropOpenCv)
//                .build();
//        setManualExposure(6, 250);
//    }
//
//    private void    setManualExposure(int exposureMS, int gain) {
//        // Wait for the camera to be open, then use the controls
//
//        if (visionPortal == null) {
//            return;
//        }
//
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        if (!isStopRequested())
//        {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//        }
//    }
//
//    public void moveRobot(double x, double y, double yaw) {
//        // Calculate wheel powers.
////testing rear camera
//
//        double leftFrontPower    =  x -y +yaw;
//        double rightFrontPower   =  x +y -yaw;
//        double leftBackPower     =  x +y +yaw;
//        double rightBackPower    =  x -y -yaw;
//
//
//        /*        good for front camera
//        double leftFrontPower    =  -x +y +yaw;
//        double rightFrontPower   =  -x -y -yaw;
//        double leftBackPower     =  -x -y +yaw;
//        double rightBackPower    =  -x +y -yaw;
//*/
//
////original set up
////        double leftFrontPower    =  x -y -yaw;
////        double rightFrontPower   =  x +y +yaw;
////        double leftBackPower     =  x +y -yaw;
////        double rightBackPower    =  x -y +yaw;
////original set up
//
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        LFMotor.setPower(leftFrontPower);
//        RFMotor.setPower(rightFrontPower);
//        LBMotor.setPower(leftBackPower);
//        RBMotor.setPower(rightBackPower);
//    }
//
//
//
////did not use for this season
////did not use for this season
//// did not use for this season
//
////start of  newGetHeading()
////   public double newGetHeading(){
////        double currentHeading =imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
////// BNO055IMU
//////double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
/////*
////
////Orientation 	getAngularOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit)
////Returns the absolute orientation of the sensor as a set three angles with indicated parameters.
////*
////*
////*
////*/
////double headingChange = currentHeading - previousHeading;
////        if(headingChange < -180){
////        headingChange += 360;
////    }else if(headingChange > 180){
////        headingChange -= 360;
////    }
////    processedHeading += headingChange;
////    previousHeading = currentHeading;
////        return processedHeading;
////}
////endof  newGetHeading()
//
////
////    //start of  goStraight()
////    public void goStraight(double power,double rotations){
////        double multiplier;
////        initialFR = RFMotor.getCurrentPosition()/ticksPerRotation;
////        initialFL = LFMotor.getCurrentPosition()/ticksPerRotation;
////        initialBR = RBMotor.getCurrentPosition()/ticksPerRotation;
////        initialBL = LBMotor.getCurrentPosition()/ticksPerRotation;
////        positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
////        positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
////        positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
////        positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
////        RFMotor.setPower(power);
////        LFMotor.setPower(power);
////        RBMotor.setPower(power);
////        LBMotor.setPower(power);
////        targetheading = getHeading();
////        while(positionFR < rotations && positionFL < rotations && positionBR < rotations &&
////                positionBL < rotations && opModeIsActive()){
////            heading = getHeading();
////            if(heading-targetheading>=0){ //to the left
////                multiplier = .1*(heading-targetheading)+1;
////                LFMotor.setPower(power*multiplier);
////                LBMotor.setPower(power*multiplier);
////                RFMotor.setPower(power);
////                RBMotor.setPower(power);
////            }else if(heading-targetheading<0){
////                multiplier = -.1*(heading-targetheading)+1;
////                RFMotor.setPower(power*multiplier);
////                RBMotor.setPower(power*multiplier);
////                LFMotor.setPower(power);
////                LBMotor.setPower(power);
////            }
////            positionFR = (RFMotor.getCurrentPosition()/ticksPerRotation)-initialFR;
////            positionFL = (LFMotor.getCurrentPosition()/ticksPerRotation)-initialFL;
////            positionBR = (RBMotor.getCurrentPosition()/ticksPerRotation)-initialBR;
////            positionBL = (LBMotor.getCurrentPosition()/ticksPerRotation)-initialBL;
////        }
////        stopMotors();
////    }
//////endof goStraight()
//
//////startof absoluteHeading
////    public void absoluteHeading(double power, double degrees){
//////As you can see, this only works if you also have the newGetHeading() and gyroTurn() functions.
//////gyroTurn() is where the loop is - where it would lock people out - so you might need
//////to copy all three functions and make changes to gyroTurn().
//////newGetHeading() should probably not cause any problems, though.
////        double current = newGetHeading(); //Set the current heading to what the heading is,accounting for
//////the whole -179 -> 180 flip
////        double processedCurrent = current % 360.0;//The processed version of the current heading
//////(This just removes extra rotations)
////        telemetry.addData("how many to turn",degrees-processedCurrent);
////        telemetry.addData("power", power); //We probably don't need any of these telemetry things
////        telemetry.update(); //But here they are
////        gyroTurn(power,degrees-processedCurrent); //This is where the actual turning bit happens.
//////It uses gyroTurn(), which you'll probably have to adapt for teleop use.
////    }
//////end of absoluteHeading
////    public double getHeading(){
////        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
////
/////*
//////// Create an object to receive the IMU angles
//////YawPitchRollAngles robotOrientation;
//////robotOrientation = imu.getRobotYawPitchRollAngles();
//////
//////// Now use these simple methods to extract each angle
//////// (Java type double) from the object you just created:
//////double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
//////double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
//////double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
////
////// Create Orientation variable
////        Orientation myRobotOrientation;
////
////// Get Robot Orientation
////        myRobotOrientation = imu.getRobotOrientation(
////                AxesReference.INTRINSIC,
////                AxesOrder.XYZ,
////                AngleUnit.DEGREES
////        );
////
////// Then read or display the desired values (Java type float):
////        float X_axis = myRobotOrientation.firstAngle;
////        float Y_axis = myRobotOrientation.secondAngle;
////        float Z_axis = myRobotOrientation.thirdAngle;
////
//////        BNO055IMU
//////        imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
//////        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
////FIRST Tech Challenge robots drive mostly on a flat playing field, typically using the IMU to monitor or control Heading (Yaw or Z-angle).
////
////*/
////    }
////public void gyroTurn(double power, double degrees){ //Fright is negative
////    if(opModeIsActive()){
////        double gyroinitial = newGetHeading();
////        if(degrees>0){ //turn left
////            while(newGetHeading() - gyroinitial < degrees && opModeIsActive()){
////                RFMotor.setPower(power);
////                LBMotor.setPower(-power);
////                LFMotor.setPower(-power);
////                RBMotor.setPower(power);
////                updates = Double.toString(newGetHeading());
////                telemetry.addData("Heading", newGetHeading());
////                telemetry.update();
////            }
////            stopMotors();
////        }
////        else{//turn right
////            while(newGetHeading() - gyroinitial > degrees && opModeIsActive()){
////                RFMotor.setPower(-power);
////                LBMotor.setPower(power);
////                LFMotor.setPower(power);
////                RBMotor.setPower(-power);
////
////                updates = Double.toString(getHeading());
////                telemetry.addData("Heading", newGetHeading());
////                telemetry.update();
////            }
////            stopMotors();
////        }
////    }
////}
//
////    public void moveForward(double power, double distanceInInch) {
////
////        distanceInInchDouble=(double)(distanceInInch*537/(Math.PI * wheelDiameterInInches));
////        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        RFMotor.setTargetPosition((int) -distanceInInchDouble);
////        RBMotor.setTargetPosition((int) -distanceInInchDouble);
////        LFMotor.setTargetPosition((int) -distanceInInchDouble);
////        LBMotor.setTargetPosition((int) -distanceInInchDouble);
////        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        RFMotor.setPower(+power);
////        RBMotor.setPower(+power);
////        LFMotor.setPower(+power);
////        LBMotor.setPower(+power);
////        while (RFMotor.isBusy() || RBMotor.isBusy() || LFMotor.isBusy() || LBMotor.isBusy() ||false) {}
////        RFMotor.setPower(0);
////        LFMotor.setPower(0);
////        RBMotor.setPower(0);
////        LBMotor.setPower(0);
////    }
//
//
//}