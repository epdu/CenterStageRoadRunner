//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//@Autonomous
//public class CarolinaAutonomousAttempt extends LinearOpMode {
//    //assuming we start on the far side as everybody prefers the close side
//    //place purple pixel on spike mark with team prop on it- 20 pts
//    //yellow pixel place with team prop indicator- 25pts
//    //backstage parking- 5 pts
//
//    DcMotor RFMotor;
//    DcMotor LFMotor;
//    DcMotor RBMotor;
//    DcMotor LBMotor;
//    private OpenCvCamera Camera;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
//        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
//        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
//        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
//
//        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
//        Camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        Camera.setPipeline(new rectangle_thresholder_pipeline());
//        Camera.openCameraDevice();
//        Camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(Camera, 30);
//
//        waitForStart();
//
//    }
//
//    public class rectangle_thresholder_pipeline extends OpenCvPipeline {
//        private String location = "nothing"; // output
//        public Scalar lower = new Scalar(0, 75, 75); // HSV threshold bounds
//        public Scalar upper = new Scalar(0, 255, 255);
//
//        private Mat hsvMat = new Mat(); // converted image
//        private Mat binaryMat = new Mat(); // imamge analyzed after thresholding
//        private Mat maskedInputMat = new Mat();
//
//        // Rectangle regions to be scanned
//        Point topLeft1 = new Point(10, 0); Point bottomRight1 = new Point(40, 20);
//        private Point topLeft2 = new Point(10, 0), bottomRight2 = new Point(40, 20);
//
//        public rectangle_thresholder_pipeline() {
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            Mat hsvFrame = new Mat();
//            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);
//
//            Core.inRange(hsvMat, lower, upper, binaryMat);
//
//            double w1 = 0, w2 = 0;
//            // process the pixel value for each rectangle  (255 = W, 0 = B)
//            for (int i = (int) topLeft1.x; i <= bottomRight1.x; i++) {
//                for (int j = (int) topLeft1.y; j <= bottomRight1.y; j++) {
//                    if (binaryMat.get(i, j)[0] == 255) {
//                        w1++;
//                    }
//                }
//            }
//            for (int i = (int) topLeft2.x; i <= bottomRight2.x; i++) {
//                for (int j = (int) topLeft2.y; j <= bottomRight2.y; j++) {
//                    if (binaryMat.get(i, j)[0] == 255) {
//                        w2++;
//                    }
//                }
//            }
//
//            // Determine object location
//            if (w1 > w2) {
//                location = "1";
//            } else if (w1 < w2) {
//                location = "2";
//            }
//
//            return binaryMat;
//        }
//
//        public String getLocation() {
//            return location;
//        }
//    }
//}
