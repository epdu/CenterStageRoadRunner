/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class implements a generic OpenCV color blob detection pipeline.
 */
public class TrcOpenCvColorBlobVisionProcessor implements VisionProcessor
{
   /**
    * This class encapsulates all the filter contour parameters.
    */
   public static class FilterContourParams
   {
      double minArea = 0.0;
      double minPerimeter = 0.0;
      double[] widthRange = {0.0, 1000.0};
      double[] heightRange = {0.0, 1000.0};
      double[] solidityRange = {0.0, 100.0};
      double[] verticesRange = {0.0, 1000000.0};
      double[] aspectRatioRange = {0.0, 1000.0};

      public FilterContourParams setMinArea(double minArea)
      {
         this.minArea = minArea;
         return this;
      }   //setMinArea

      public FilterContourParams setMinPerimeter(double minPerimeter)
      {
         this.minPerimeter = minPerimeter;
         return this;
      }   //setMinPerimeter

      public FilterContourParams setWidthRange(double min, double max)
      {
         this.widthRange[0] = min;
         this.widthRange[1] = max;
         return this;
      }   //setWidthRange

      public FilterContourParams setHeightRange(double min, double max)
      {
         this.heightRange[0] = min;
         this.heightRange[1] = max;
         return this;
      }   //setHeightRange

      public FilterContourParams setSolidityRange(double min, double max)
      {
         this.solidityRange[0] = min;
         this.solidityRange[1] = max;
         return this;
      }   //setSolidityRange

      public FilterContourParams setVerticesRange(double min, double max)
      {
         this.verticesRange[0] = min;
         this.verticesRange[1] = max;
         return this;
      }   //setVerticesRange

      public FilterContourParams setAspectRatioRange(double min, double max)
      {
         this.aspectRatioRange[0] = min;
         this.aspectRatioRange[1] = max;
         return this;
      }   //setAspectRatioRange

      @Override
      public String toString()
      {
         return String.format(
             Locale.US,
             "minArea=%f,minPerim=%f,width=(%f,%f),height=(%f,%f),solidity=(%f,%f),vertices=(%f,%f)," +
             "aspectRatio=(%f,%f)",
             minArea, minPerimeter, widthRange[0], widthRange[1], heightRange[0], heightRange[1], solidityRange[0],
             solidityRange[1], verticesRange[0], verticesRange[1], aspectRatioRange[0], aspectRatioRange[1]);
      }   //toString

   }   //class FilterContourParams

   private static final int DEF_LINE_COLOR = Color.GREEN;
   private static final float DEF_LINE_WIDTH = 4.0f;
   private static final int DEF_TEXT_COLOR = Color.RED;
   private static final float DEF_TEXT_SIZE = 20.0f;
   private final Paint linePaint;
   private final Paint textPaint;

   private final String label;
   private final Integer colorConversion;
   private final Scalar lowColorThresholds;
   private final Scalar highColorThresholds;
   private final FilterContourParams filterContourParams;
   private final boolean externalContourOnly;
   private final Mat colorConversionOutput = new Mat();
   private final Mat colorThresholdOutput = new Mat();
   private final Mat morphologyOutput = new Mat();
   private final Mat hierarchy = new Mat();

   private final AtomicReference<Rect[]> detectedObjectsUpdate = new AtomicReference<>();
   private int morphOp = Imgproc.MORPH_CLOSE;
   private Mat kernelMat = null;
   /**
    * Constructor: Create an instance of the object.
    *
    * @param label specifies the label for the detected object.
    * @param colorConversion specifies color space conversion, can be null if no color space conversion.
    *        Note: FTC ECOV input Mat format is RGBA, so you need to do Imgproc.COLOR_RGBA2xxx or
    *        Imgproc.COLOR_RGB2xxx conversion. For FRC, the Desktop OpenCV input Mat format is BGRA, so you need to
    *        do Imgproc.COLOR_BGRAxxx or Imgproc.COLOR_BGR2xxx conversion.
    * @param lowColorThresholds specifies the low color thresholds.
    * @param highColorThresholds specifies the high color thresholds.
    * @param filterContourParams specifies the parameters for filtering contours, can be null if not provided.
    * @param externalContourOnly specifies true for finding external contours only, false otherwise (not applicable
    *        if filterContourParams is null).
    */
   public TrcOpenCvColorBlobVisionProcessor(
       String label, Integer colorConversion, Scalar lowColorThresholds, Scalar highColorThresholds,
       FilterContourParams filterContourParams, boolean externalContourOnly)
   {
      if (lowColorThresholds == null || highColorThresholds == null)
      {
         throw new RuntimeException("Must provide color thresholds.");
      }

      this.label = label;
      this.colorConversion = colorConversion;
      this.lowColorThresholds = lowColorThresholds;
      this.highColorThresholds = highColorThresholds;
      this.filterContourParams = filterContourParams;
      this.externalContourOnly = externalContourOnly;

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
   }   //TrcOpenCvColorBlobPipeline

   /**
    * This method returns the instance name.
    *
    * @return instance name.
    */
   @Override
   public String toString()
   {
      return label;
   }   //toString

   /**
    * This method returns the list of detected object rects.
    *
    * @return array of detected object rects.
    */
   public Rect[] getDetectedObjects()
   {
      return detectedObjectsUpdate.getAndSet(null);
   }   //getDetectedObjects

   /**
    * This method enables Morphology operation in the pipeline with the specifies kernel shape and size.
    *
    * @param morphOp specifies the Morphology operation.
    * @param kernelShape specifies the kernel shape.
    * @param kernelSize specifies the kernel size.
    */
   public void setMorphologyOp(int morphOp, int kernelShape, Size kernelSize)
   {
      if (kernelMat != null)
      {
         // Release an existing kernel mat if there is one.
         kernelMat.release();
      }
      this.morphOp = morphOp;
      kernelMat = Imgproc.getStructuringElement(kernelShape, kernelSize);
   }   //setMorphologyOp

   /**
    * This method enables Morphology operation in the pipeline with default kernel shape and size.
    *
    * @param morphOp specifies the Morphology operation.
    */
   public void setMorphologyOp(int morphOp)
   {
      setMorphologyOp(morphOp, Imgproc.MORPH_ELLIPSE, new Size(5, 5));
   }   //setMorphologyOp

   /**
    * This method enables Morphology operation in the pipeline with default kernel shape and size.
    */
   public void setMorphologyOp()
   {
      setMorphologyOp(Imgproc.MORPH_CLOSE, Imgproc.MORPH_ELLIPSE, new Size(5, 5));
   }   //setMorphologyOp

   /**
    * This method filters out contours that do not meet certain criteria.
    *
    * @param inputContours specifies the input list of contours.
    * @param filterContourParams specifies the filter contour parameters.
    * @param output specifies the the output list of contours.
    */
   private void filterContours(
       List<MatOfPoint> inputContours, FilterContourParams filterContourParams, List<MatOfPoint> output)
   {
      final MatOfInt hull = new MatOfInt();
      output.clear();
      //
      // Perform the filtering.
      //
      for (int i = 0; i < inputContours.size(); i++)
      {
         final MatOfPoint contour = inputContours.get(i);
         final Rect bb = Imgproc.boundingRect(contour);
         // Check width.
         if (bb.width < filterContourParams.widthRange[0] || bb.width > filterContourParams.widthRange[1])
         {
            continue;
         }
         // Check height.
         if (bb.height < filterContourParams.heightRange[0] || bb.height > filterContourParams.heightRange[1])
         {
            continue;
         }
         // Check area.
         final double area = Imgproc.contourArea(contour);
         if (area < filterContourParams.minArea)
         {
            continue;
         }
         // Check perimeter.
         if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < filterContourParams.minPerimeter)
         {
            continue;
         }
         // Check solidity.
         Imgproc.convexHull(contour, hull);
         MatOfPoint mopHull = new MatOfPoint();
         mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
         for (int j = 0; j < hull.size().height; j++)
         {
            int index = (int)hull.get(j, 0)[0];
            double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
            mopHull.put(j, 0, point);
         }
         final double solid = 100 * area / Imgproc.contourArea(mopHull);
         if (solid < filterContourParams.solidityRange[0] || solid > filterContourParams.solidityRange[1])
         {
            continue;
         }
         // Check vertex count.
         if (contour.rows() < filterContourParams.verticesRange[0] ||
             contour.rows() > filterContourParams.verticesRange[1])
         {
            continue;
         }
         // Check aspect ratio.
         final double ratio = bb.width / (double)bb.height;
         if (ratio < filterContourParams.aspectRatioRange[0] || ratio > filterContourParams.aspectRatioRange[1])
         {
            continue;
         }

         output.add(contour);
      }
   }   //filterContours

   //
   // Implements VisionProcessor interface.
   //

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
      Rect[] detectedRects = null;
      ArrayList<MatOfPoint> contoursOutput = new ArrayList<>();
      ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

      // Do color space conversion.
      if (colorConversion != null)
      {
         Imgproc.cvtColor(input, colorConversionOutput, colorConversion);
         input = colorConversionOutput;
      }

      // Do color filtering.
      Core.inRange(input, lowColorThresholds, highColorThresholds, colorThresholdOutput);
      input = colorThresholdOutput;

      // Do morphology.
      if (kernelMat != null)
      {
         Imgproc.morphologyEx(input, morphologyOutput, morphOp, kernelMat);
         input = morphologyOutput;
      }

      // Find contours.
      Imgproc.findContours(
          input, contoursOutput, hierarchy, externalContourOnly? Imgproc.RETR_EXTERNAL: Imgproc.RETR_LIST,
          Imgproc.CHAIN_APPROX_SIMPLE);

      // Do contour filtering.
      if (filterContourParams != null)
      {
         filterContours(contoursOutput, filterContourParams, filterContoursOutput);
         contoursOutput = filterContoursOutput;
      }

      if (contoursOutput.size() > 0)
      {
         detectedRects = new Rect[contoursOutput.size()];
         for (int i = 0; i < detectedRects.length; i++)
         {
            detectedRects[i] = Imgproc.boundingRect(contoursOutput.get(i));
         }
         detectedObjectsUpdate.set(detectedRects);
      }

      return detectedRects;
   }   //processFrame

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
         Rect[] detectedObjects = (Rect[]) userContext;
         for (Rect objRect: detectedObjects)
         {
            // Detected rect is on camera Mat that has different resolution from the canvas. Therefore, we must
            // scale the rect to canvas resolution.
            float left = objRect.x * scaleBmpPxToCanvasPx;
            float right = (objRect.x + objRect.width) * scaleBmpPxToCanvasPx;
            float top = objRect.y * scaleBmpPxToCanvasPx;
            float bottom = (objRect.y + objRect.height) * scaleBmpPxToCanvasPx;

            canvas.drawLine(left, top, right, top, linePaint);
            canvas.drawLine(right, top, right, bottom, linePaint);
            canvas.drawLine(right, bottom, left, bottom, linePaint);
            canvas.drawLine(left, bottom, left, top, linePaint);
            canvas.drawText(label, left, bottom, textPaint);
         }
      }
   }   //onDrawFrame

}  //class TrcOpenCvColorBlobVisionProcessor
