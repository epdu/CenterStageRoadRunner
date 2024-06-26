//package org.wolfcorp.cv.tutorial;
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
/*
    static final Rect CENTER_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));
*/
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public SkystoneDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);


        Scalar lowHSV = new Scalar(123, 25, 31); // lower bound HSV for blue tested by cone 223 25 31
        Scalar highHSV =  new Scalar(143, 255, 255); // higher bound HSV for blue

/*
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

*/

/*
        Scalar lowHSV = new Scalar(100, 100, 100);  // lower bound HSV for red
        Scalar highHSV =  new Scalar(180, 255, 255);  // higher bound HSV for red
*/

/*        Scalar lowHSV = new Scalar(95, 110, 50); // lower bound HSV for blue
        Scalar highHSV =  new Scalar(150, 245, 255); // higher bound HSV for blue
 */
/*
        Scalar lowHSV = new Scalar(89, 67, 61); // lower bound HSV for green
        Scalar highHSV = new Scalar(83, 43, 83); // higher bound HSV for green
*/

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft && stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        }
        else if (stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}