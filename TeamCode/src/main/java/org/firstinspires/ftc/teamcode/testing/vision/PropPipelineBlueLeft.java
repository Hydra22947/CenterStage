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
public class PropPipelineBlueLeft extends OpenCvPipeline {

    // blue , not seeing the right line
    public static int leftX = 15, leftY = 80;
    public static int centerX = 410, centerY = 65;

    double avgLeft = 0, avgCenter = 0;
    // red, not seeing the left line

    public static double MIN_PIXELS = 1500;


    public static int widthLeft = 110, heightLeft = 125;
    public static int widthCenter = 140, heightCenter = 140;

    public static int redMinH = 0;
    public static int redMinS = 50;
    public static int redMinV = 0;
    public static int redMaxH = 70;
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


    public PropPipelineBlueLeft()
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
        Mat matLeft = workingMatrix.submat(leftY, heightLeft + leftY, leftX, leftX + widthLeft);
        Mat matCenter = workingMatrix.submat(centerY, heightCenter + centerY, centerX, centerX + widthCenter);

        // Draw rectangles around regions of interest
        Imgproc.rectangle(workingMatrix, new Rect(leftX, leftY, widthLeft, heightLeft), new Scalar(255, 255, 255));
        Imgproc.rectangle(workingMatrix, new Rect(centerX, centerY, widthCenter, heightCenter), new Scalar(255, 255, 255));

        // Calculate the average intensity of blue color in each region
        avgLeft = Core.countNonZero(matLeft);
        avgCenter = Core.countNonZero(matCenter);

        // Find the region with the maximum average blue intensity
        if(avgLeft < MIN_PIXELS && avgCenter < MIN_PIXELS)
        {
            location = Location.Right;
        }
        else if (avgLeft > MIN_PIXELS && avgLeft > avgCenter) {
            location = Location.Left;
        } else if (avgCenter > MIN_PIXELS && avgCenter > avgLeft) {
            location = Location.Center;
        }

        workingMatrix.copyTo(returnMatrix);

        return returnMatrix;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public double getAvgLeft() {
        return avgLeft;
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