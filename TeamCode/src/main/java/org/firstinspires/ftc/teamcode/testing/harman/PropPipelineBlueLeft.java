package org.firstinspires.ftc.teamcode.testing.harman;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.auto.AutoSettings;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class PropPipelineBlueLeft extends OpenCvPipeline {

    // blue , not seeing the right line
    public static int leftX = 0, leftY = 55;
    public static int centerX = 280, centerY = 20;

    double avgLeft = 0, avgCenter = 0;
    // red, not seeing the left line

    public static double NO_PROP = 85;


    public static int widthLeft = 175, heightLeft = 250;
    public static int widthCenter = 350, heightCenter = 200;

    public static int blueMinH = 40;
    public static int blueMinS = 73;
    public static int blueMinV = 0;
    public static int blueMaxH = 179;
    public static int blueMaxS = 255;
    public static int blueMaxV = 255;
    private Mat workingMatrix = new Mat();
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
        input.copyTo(workingMatrix);

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_BGR2HSV); // Convert to HSV color space

        // Define the range of blue color in HSV
        Scalar blueMin = new Scalar(blueMinH, blueMinS, blueMinV);
        Scalar blueMax = new Scalar(blueMaxH, blueMaxS, blueMaxV);

        // Threshold the HSV image to get only blue colors
        Core.inRange(workingMatrix, blueMin, blueMax, workingMatrix);

        // Perform bitwise AND operation to isolate blue regions in the input image
        Core.bitwise_and(input, input, workingMatrix);

        // Define regions of interest
        Mat matLeft = workingMatrix.submat(leftY, heightLeft + leftY, leftX, leftX + widthLeft);
        Mat matCenter = workingMatrix.submat(centerY, heightCenter + centerY, centerX, centerX + widthCenter);

        // Draw rectangles around regions of interest
        Imgproc.rectangle(workingMatrix, new Rect(leftX, leftY, widthLeft, heightLeft), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(centerX, centerY, widthCenter, heightCenter), new Scalar(0, 255, 0));

        // Calculate the average intensity of blue color in each region
        avgLeft = Core.mean(matLeft).val[0]; // Blue channel intensity
        avgCenter = Core.mean(matCenter).val[0];


        // Find the region with the maximum average blue intensity
        if(avgLeft > NO_PROP && avgCenter > NO_PROP)
        {
            location = Location.Right;
        }
        else if (avgLeft > avgCenter) {
            location = Location.Center;
        } else if (avgCenter > avgLeft) {
            location = Location.Left;
        }

        return workingMatrix;
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
}