package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;


import java.lang.Object;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint2f;
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
public class ShapeDetection {
    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        Mat img = Imgcodecs.imread("shapes.jpg");
        Mat gray = new Mat();
        Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);

        Mat thresh = new Mat();
        Imgproc.threshold(gray, thresh, 50, 255, 0);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        System.out.println("Number of contours detected: " + contours.size());

        for (MatOfPoint cnt : contours) {
            double[] point = cnt.get(0, 0);
            double x1 = point[0];
            double y1 = point[1];

            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(cnt.toArray()), approx, 0.01 * Imgproc.arcLength(new MatOfPoint2f(cnt.toArray()), true), true);

            if (approx.rows() == 4) {
                Rect rect = Imgproc.boundingRect(cnt);
                double ratio = (double) rect.width / rect.height;

                if (ratio >= 0.9 && ratio <= 1.1) {
                    Imgproc.drawContours(img, contours, contours.indexOf(cnt), new Scalar(0, 255, 255), 3);
//                    Imgproc.putText(img, "Square", new Point(x1, y1), Core.FONT_HERSHEY_PLAIN, 0.6, new Scalar(255, 255, 0), 2);
                    Imgproc.putText(img, "Square", new Point(x1, y1), Core.BORDER_DEFAULT,0.6, new Scalar(255, 255, 0), 2);
                } else {
//                    Imgproc.putText(img, "Rectangle", new Point(x1, y1), Core.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(0, 255, 0), 2);
                    Imgproc.putText(img, "Rectangle", new Point(x1, y1), Core.BORDER_DEFAULT,0.6, new Scalar(0, 255, 0), 2);
                    Imgproc.drawContours(img, contours, contours.indexOf(cnt), new Scalar(0, 255, 0), 3);
                }
            }
        }

        Imgcodecs.imwrite("Shapes_output.jpg", img);
/*        Imgcodecs.imshow("Shapes", img);
        Imgcodecs.waitKey(0);
        Imgcodecs.destroyAllWindows();
*/
/*        displayImage(img, "Image Display");

        private static void displayImage(BufferedImage img, String title) {
            JFrame frame = new JFrame(title);
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setSize(400, 400);

            ImageIcon icon = new ImageIcon(img);
            JLabel label = new JLabel(icon);
            frame.add(label);

            frame.setVisible(true);
        }

 */
    }
}
