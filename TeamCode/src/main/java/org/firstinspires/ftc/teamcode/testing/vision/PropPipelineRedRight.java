package org.firstinspires.ftc.teamcode.testing.vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.CvType;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class PropPipelineRedRight extends OpenCvPipeline {

    // blue , not seeing the right line
    public static int rightX = 440, rightY = 50;
    public static int centerX = 0, centerY = 80;

    double avgRight = 0, avgCenter = 0;
    // red, not seeing the left line

    public static double MIN_PIXELS = 1500;


    public static int widthRight = 160, heightRight = 160;
    public static int widthCenter = 150, heightCenter = 130;

    public static int redMinH = 120;
    public static int redMinS = 120;
    public static int redMinV = 70;
    public static int redMaxH = 150;
    public static int redMaxS = 255;
    public static int redMaxV = 255;
    public static int idkNumber = 10;
    private Mat workingMatrix = new Mat();
    private Mat returnMatrix = new Mat();
    public enum Location
    {
        Left,
        Right,
        Center
    }

    Location location = Location.Center;


    public PropPipelineRedRight()
    {
    }

    @Override
    public final Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, workingMatrix, Imgproc.COLOR_BGR2HSV); // Convert to HSV color space

        Mat kernel = Mat.ones(idkNumber,idkNumber, CvType.CV_32F);

        // Define the range of blue color in HSV
        Scalar redMin = new Scalar(redMinH, redMinS, redMinV);
        Scalar redMax = new Scalar(redMaxH, redMaxS, redMaxV);

        // Threshold the HSV image to get only blue colors
        Core.inRange(workingMatrix, redMin, redMax, workingMatrix);

        // Perform bitwise AND operation to isolate blue regions in the input image
        Imgproc.morphologyEx(workingMatrix, workingMatrix, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(workingMatrix, workingMatrix, Imgproc.MORPH_CLOSE, kernel);

        // Define regions of interest
        Mat matRight = workingMatrix.submat(rightY, heightRight + rightY, rightX, rightX + widthRight);
        Mat matCenter = workingMatrix.submat(centerY, heightCenter + centerY, centerX, centerX + widthCenter);

        // Draw rectangles around regions of interest
        Imgproc.rectangle(workingMatrix, new Rect(rightX, rightY, widthRight, heightRight), new Scalar(255, 255, 255));
        Imgproc.rectangle(workingMatrix, new Rect(centerX, centerY, widthCenter, heightCenter), new Scalar(255, 255, 255));

        // Calculate the average intensity of blue color in each region
        avgRight = Core.countNonZero(matRight);
        avgCenter = Core.countNonZero(matCenter);

        // Find the region with the maximum average blue intensity
        if(avgRight < MIN_PIXELS && avgCenter < MIN_PIXELS)
        {
            location = Location.Left;
        }
        else if (avgRight > MIN_PIXELS && avgRight > avgCenter) {
            location = Location.Right;
        } else if (avgCenter > MIN_PIXELS && avgCenter > avgRight) {
            location = Location.Center;
        }

        workingMatrix.copyTo(returnMatrix);

        return returnMatrix;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public double getAvgRight() {
        return avgRight;
    }

    public double getAvgCenter() {
        return avgCenter;
    }

    public Location getLocation() {
        return location;
    }

    public static void setMinPixels(double noProp) {
        MIN_PIXELS = noProp;
    }

    public static double getMinPixels() {
        return MIN_PIXELS;
    }
}