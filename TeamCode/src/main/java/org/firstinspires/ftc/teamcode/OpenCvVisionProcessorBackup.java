package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import java.util.ArrayList;
import java.util.List;
//Good version 02082024
public class OpenCvVisionProcessorBackup implements VisionProcessor {
    private static final int DEF_LINE_COLOR = Color.GREEN;
    private static final float DEF_LINE_WIDTH = 4.0f;
    private static final int DEF_TEXT_COLOR = Color.RED;
    private static final float DEF_TEXT_SIZE = 20.0f;
    private final Paint linePaint;
    private final Paint textPaint;
    private final String name;
    private final Scalar lowHSV;
    private final Scalar highHSV;
    private final Point teamPropCentroid = new Point();
    private final Mat hsvFrame = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    public OpenCvVisionProcessor(String name, Scalar lowHSV, Scalar highHSV)
    {
        this.name = name;
        this.lowHSV = lowHSV;
        this.highHSV = highHSV;

        linePaint = new Paint();
        linePaint.setAntiAlias(true);
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setColor(DEF_LINE_COLOR);
        linePaint.setStrokeWidth(DEF_LINE_WIDTH);

        textPaint = new Paint();
        textPaint.setAntiAlias(true);
        textPaint.setTextAlign(Paint.Align.LEFT);
        textPaint.setColor(DEF_TEXT_COLOR);
        textPaint.setTextSize(DEF_TEXT_SIZE);
    }

    /**
     * This method is called to initialize the vision processor.
     *
     * @param width specifies the image width.
     * @param height specifies the image height.
     * @param calibration specifies the camera calibration data.
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        // Don't really need to do anything here.
    }   //init

    /**
     * This method is called to process an image frame.
     *
     * @param input specifies the source image to be processed.
     * @param captureTimeNanos specifies the capture frame timestamp.
     * @return array of detected objects.
     */
    @Override
    public Object processFrame(Mat input, long captureTimeNanos)
    {
        // Preprocess the frame to detect yellow regions
        preprocessFrame(input);
        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Find the largest yellow contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);
        if (largestContour != null)
        {
            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            teamPropCentroid.x = moments.get_m10()/moments.get_m00();
            teamPropCentroid.y = moments.get_m01()/moments.get_m00();
        }

        return largestContour;
    }   //processFrame

    public Point getTeamPropCentroid()
    {
        return teamPropCentroid;
    }

    /**
     * Called during the viewport's frame rendering operation at some later point during processFrame(). Allows you
     * to use the Canvas API to draw annotations on the frame, rather than using OpenCV calls. This allows for more
     * eye-candy annotations since you've got a high resolution canvas to work with rather than, say, a 320x240 image.
     * <p>
     * Note that this is NOT called from the same thread that calls processFrame(), and may actually be called from
     * the UI thread depending on the viewport renderer.
     * </p>
     *
     * @param canvas the canvas that's being drawn on NOTE: Do NOT get dimensions from it, use below
     * @param onscreenWidth the width of the canvas that corresponds to the image
     * @param onscreenHeight the height of the canvas that corresponds to the image
     * @param scaleBmpPxToCanvasPx multiply pixel coords by this to scale to canvas coords
     * @param scaleCanvasDensity a scaling factor to adjust e.g. text size. Relative to Nexus5 DPI.
     * @param userContext whatever you passed in when requesting the draw hook :monkey:
     */
    @Override
    public synchronized void onDrawFrame(
            Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
            Object userContext)
    {
        // Allow only one draw operation at a time (we could be called from two different threads - viewport or
        // camera stream).
        if (userContext != null)
        {
            MatOfPoint largestContour = (MatOfPoint) userContext;
            Rect boundingRect = Imgproc.boundingRect(largestContour);
            // Detected rect is on camera Mat that has different resolution from the canvas. Therefore, we must
            // scale the rect to canvas resolution.
            float left = boundingRect.x * scaleBmpPxToCanvasPx;
            float right = (boundingRect.x + boundingRect.width) * scaleBmpPxToCanvasPx;
            float top = boundingRect.y * scaleBmpPxToCanvasPx;
            float bottom = (boundingRect.y + boundingRect.height) * scaleBmpPxToCanvasPx;

            canvas.drawLine(left, top, right, top, linePaint);
            canvas.drawLine(right, top, right, bottom, linePaint);
            canvas.drawLine(right, bottom, left, bottom, linePaint);
            canvas.drawLine(left, bottom, left, top, linePaint);
            canvas.drawText(name, left, bottom, textPaint);
        }
    }   //onDrawFrame

    private void preprocessFrame(Mat frame) {
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvFrame, lowHSV, highHSV, yellowMask);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);
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

}