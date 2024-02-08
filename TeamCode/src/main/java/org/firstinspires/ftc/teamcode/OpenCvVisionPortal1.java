//package org.firstinspires.ftc.teamcode;
//
//
//import android.graphics.Canvas;
//import android.graphics.Color;
//import android.graphics.Paint;
//
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class OpenCvVisionProcessor1 implements VisionProcessor {
//    private static final int DEF_LINE_COLOR = Color.GREEN;
//    private static final float DEF_LINE_WIDTH = 4.0f;
//    private static final int DEF_TEXT_COLOR = Color.RED;
//    private static final float DEF_TEXT_SIZE = 20.0f;
//    private final Paint linePaint;
//    private final Paint textPaint;
//    private String name;
//    private Scalar lowHSV;
//    private Scalar highHSV;
//    private Point teamPropCentroid = new Point();
//    public OpenCvVisionProcessor1(String name, Scalar lowHSV, Scalar highHSV)
//    {
//        this.name = name;
//        this.lowHSV = lowHSV;
//        this.highHSV = highHSV;
//        linePaint = new Paint();
////
//        linePaint.setAntiAlias(true);
//        linePaint.setStrokeCap(Paint.Cap.ROUND);
//        linePaint.setColor(DEF_LINE_COLOR);
//        linePaint.setStrokeWidth(DEF_LINE_WIDTH);
//
//        textPaint = new Paint();
//        textPaint.setAntiAlias(true);
//        textPaint.setTextAlign(Paint.Align.LEFT);
//        textPaint.setColor(DEF_TEXT_COLOR);
//        textPaint.setTextSize(DEF_TEXT_SIZE);
//    }
//
//    /**
//     * This method is called to initialize the vision processor.
//     *
//     * @param width specifies the image width.
//     * @param height specifies the image height.
//     * @param calibration specifies the camera calibration data.
//     */
//    @Override
//    public void init(int width, int height, CameraCalibration calibration)
//    {
//        // Don't really need to do anything here.
//    }   //init
//
//    /**
//     * This method is called to process an image frame.
//     *
//     * @param input specifies the source image to be processed.
//     * @param captureTimeNanos specifies the capture frame timestamp.
//     * @return array of detected objects.
//     */
//    @Override
//    public Object processFrame(Mat input, long captureTimeNanos)
//    {
//        // Preprocess the frame to detect yellow regions
//        Mat yellowMask = preprocessFrame(input);
//        // Find contours of the detected yellow regions
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//        // Find the largest yellow contour (blob)
//        MatOfPoint largestContour = findLargestContour(contours);
//        // Calculate the centroid of the largest contour
//        Moments moments = Imgproc.moments(largestContour);
//        teamPropCentroid.x = moments.get_m10() / moments.get_m00();
//        teamPropCentroid.y = moments.get_m01() / moments.get_m00();
///*       if (largestContour != null) {
//            // Draw a red outline around the largest detected object
////            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
//            // Calculate the width of the bounding box
//            width = calculateWidth(largestContour);
//            // Display the width next to the label
//            String widthLabel = "Width: " + (int) width + " pixels";
////            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//            //Display the Distance
//            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
////            Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//            // Calculate the centroid of the largest contour
//            Moments moments = Imgproc.moments(largestContour);
//            cX = moments.get_m10() / moments.get_m00();
//            cY = moments.get_m01() / moments.get_m00();
//            // Draw a dot at the centroid
//            String label = "(" + (int) cX + ", " + (int) cY + ")";
////            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
////            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
//        }
//*/
//        return largestContour;
//    }   //processFrame
//
//    public Point getTeamPropCentroid()
//    {
//        return teamPropCentroid;
//    }
//
//    /**
//     * Called during the viewport's frame rendering operation at some later point during processFrame(). Allows you
//     * to use the Canvas API to draw annotations on the frame, rather than using OpenCV calls. This allows for more
//     * eye-candy annotations since you've got a high resolution canvas to work with rather than, say, a 320x240 image.
//     * <p>
//     * Note that this is NOT called from the same thread that calls processFrame(), and may actually be called from
//     * the UI thread depending on the viewport renderer.
//     * </p>
//     *
//     * @param canvas the canvas that's being drawn on NOTE: Do NOT get dimensions from it, use below
//     * @param onscreenWidth the width of the canvas that corresponds to the image
//     * @param onscreenHeight the height of the canvas that corresponds to the image
//     * @param scaleBmpPxToCanvasPx multiply pixel coords by this to scale to canvas coords
//     * @param scaleCanvasDensity a scaling factor to adjust e.g. text size. Relative to Nexus5 DPI.
//     * @param userContext whatever you passed in when requesting the draw hook :monkey:
//     */
//    @Override
//    public synchronized void onDrawFrame(
//            Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
//            Object userContext)
//    {
//        // Allow only one draw operation at a time (we could be called from two different threads - viewport or
//        // camera stream).
//        if (userContext != null)
//        {
//            MatOfPoint largestContour = (MatOfPoint) userContext;
//            Rect boundingRect = Imgproc.boundingRect(largestContour);
//            // Detected rect is on camera Mat that has different resolution from the canvas. Therefore, we must
//            // scale the rect to canvas resolution.
//            float left = boundingRect.x * scaleBmpPxToCanvasPx;
//            float right = (boundingRect.x + boundingRect.width) * scaleBmpPxToCanvasPx;
//            float top = boundingRect.y * scaleBmpPxToCanvasPx;
//            float bottom = (boundingRect.y + boundingRect.height) * scaleBmpPxToCanvasPx;
//
//            canvas.drawLine(left, top, right, top, linePaint);
//            canvas.drawLine(right, top, right, bottom, linePaint);
//            canvas.drawLine(right, bottom, left, bottom, linePaint);
//            canvas.drawLine(left, bottom, left, top, linePaint);
//            canvas.drawText(name, left, bottom, textPaint);
//        }
//    }   //onDrawFrame
//
//    private Mat preprocessFrame(Mat frame) {
//        Mat hsvFrame = new Mat();
//        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);
//
////change HSV value for different team prop
////        Scalar lowHSV = new Scalar(1, 98, 34); // lower bound HSV for blue tested by blue cone 223 25 31
////        Scalar highHSV = new Scalar(30, 255, 255);
//
//        // Scalar lowHSV = new Scalar(1, 60, 58); // lower bound HSV for red tested by red team prop
//        //  Scalar highHSV = new Scalar(10, 255, 255);
////
///*
//            Scalar lowHSV = new Scalar(123, 25, 31); // lower bound HSV for blue tested by cone 223 25 31
//            Scalar highHSV =  new Scalar(143, 255, 255); // higher bound HSV for blue  214, 34, 28       100-140
//
//            Scalar lowHSV = new Scalar(1, 98, 34); // lower bound HSV for red tested by cone 10, 98, 34
//            Scalar highHSV =  new Scalar(20, 255, 255); // higher bound HSV for red
//
//            Scalar lowHSV = new Scalar(200, 32, 49); // lower bound HSV for green tested by pixel 111, 32, 49
//            Scalar highHSV =  new Scalar(121, 255, 255); // higher bound HSV for green
//
//*/
//
//        Mat yellowMask = new Mat();
//        Core.inRange(hsvFrame, lowHSV, highHSV, yellowMask);
//
//        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
//        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
//        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);
//
//        return yellowMask;
//    }
//
//    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
//        double maxArea = 0;
//        MatOfPoint largestContour = null;
//
//        for (MatOfPoint contour : contours) {
//            double area = Imgproc.contourArea(contour);
//            if (area > maxArea) {
//                maxArea = area;
//                largestContour = contour;
//            }
//        }
//
//        return largestContour;
//    }
//
//}