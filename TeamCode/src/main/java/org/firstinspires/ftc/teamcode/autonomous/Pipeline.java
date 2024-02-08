package org.firstinspires.ftc.teamcode.autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {

    public int allianceColor = 0;

    private String location = "nothing"; // output

    public Scalar lower;
    public Scalar upper;
    public Scalar lower1 = new Scalar(0, 0, 0); // RED
    public Scalar upper1 = new Scalar(255, 255, 255);

    public Scalar lower2 = new Scalar(0, 0, 0); // BLUE
    public Scalar upper2 = new Scalar(255, 255, 255);

    private Mat hsvMat = new Mat(); // converted image
    private Mat binaryMat = new Mat(); // imamge analyzed after thresholding
    private Mat maskedInputMat = new Mat();

    // Rectangle regions to be scanned
    private Point topLeft1 = new Point(10, 0), bottomRight1 = new Point(40, 20);
    private Point topLeft2 = new Point(40, 0), bottomRight2 = new Point(80, 20);

    private Point topLeft3 = new Point(80, 0), bottomRight3 = new Point(120, 20);

    public Pipeline() {

    }

    @Override
    public Mat processFrame(Mat input) {
        if (allianceColor == 0) //RED
        {
            lower = lower1;
            upper = upper1;
        } else if (allianceColor == 1) //BLUE
        {
            lower = lower2;
            upper = upper2;
        }

        // Convert from BGR to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lower, upper, binaryMat);


        // Scan both rectangle regions, keeping track of how many
        // pixels meet the threshold value, indicated by the color white
        // in the binary image
        double w1 = 0, w2 = 0, w3 = 0;
        // process the pixel value for each rectangle  (255 = W, 0 = B)
        for (int i = (int) topLeft1.x; i <= bottomRight1.x; i++) {
            for (int j = (int) topLeft1.y; j <= bottomRight1.y; j++) {
                if (binaryMat.get(i, j)[0] == 255) {
                    w1++;
                }
            }
        }

        for (int i = (int) topLeft2.x; i <= bottomRight2.x; i++) {
            for (int j = (int) topLeft2.y; j <= bottomRight2.y; j++) {
                if (binaryMat.get(i, j)[0] == 255) {
                    w2++;
                }
            }
        }

        for (int i = (int) topLeft3.x; i <= bottomRight3.x; i++) {
            for (int j = (int) topLeft3.y; j <= bottomRight3.y; j++) {
                if (binaryMat.get(i, j)[0] == 255) {
                    w3++;
                }
            }
        }

        // Determine object location
        if (w1 > w2 && w1 > w3) {
            location = "Left";
        } else if (w2 > w1 && w2 > w3) {
            location = "Center";
        } else if (w3 > w1 && w3 > w2) {
            location = "Right";
        }

        return input;
    }

    public String getLocation() {
        return location;
    }
}
