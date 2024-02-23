package org.firstinspires.ftc.teamcode.testing.vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class PropPipelineRedLeft extends OpenCvPipeline {

    // blue , not seeing the right line
    public static int leftX = 0, leftY = 60;
    public static int centerX = 270, centerY = 20;

    double avgLeft = 0, avgCenter = 0;
    // red, not seeing the left line

    public static double NO_PROP = 100;



    public static int widthCenter = 370, heightCenter = 200;
    public static int widthLeft = 190, heightLeft = 200;

    public static int redMinH = 0;
    public static int redMinS = 104;
    public static int redMinV = 0;
    public static int redMaxH = 179;
    public static int redMaxS = 255;
    public static int redMaxV = 255;
    private Mat workingMatrix = new Mat();

    public enum Location
    {
        Left,
        Right,
        Center
    }

    Location location = Location.Center;


    public PropPipelineRedLeft()
    {
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_BGR2HSV); // Convert to HSV color space

        // Define the range of blue color in HSV
        Scalar redMin = new Scalar(redMinH, redMinS, redMinV);
        Scalar redMax = new Scalar(redMaxH, redMaxS, redMaxV);

        // Threshold the HSV image to get only blue colors
        Core.inRange(workingMatrix, redMin, redMax, workingMatrix);

        // Perform bitwise AND operation to isolate blue regions in the input image
        Core.bitwise_and(input, input, workingMatrix);

        // Define regions of interest
        Mat matCenter = workingMatrix.submat(centerY, heightCenter + centerY, centerX, centerX + widthCenter);
         Mat matRight = workingMatrix.submat(leftY, heightLeft + leftY, leftX, leftY + widthLeft);

        // Draw rectangles around regions of interest
        Imgproc.rectangle(workingMatrix, new Rect(centerX, centerY, widthCenter, heightCenter), new Scalar(0, 255, 0));
         Imgproc.rectangle(workingMatrix, new Rect(leftX, leftY, widthLeft, heightLeft), new Scalar(0, 255, 0));

        // Calculate the average intensity of blue color in each region
        avgCenter = Core.mean(matCenter).val[2];
        avgLeft = Core.mean(matRight).val[2];


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