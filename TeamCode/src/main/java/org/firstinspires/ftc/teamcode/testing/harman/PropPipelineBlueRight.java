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
public class PropPipelineBlueRight extends OpenCvPipeline {

    // blue , not seeing the right line
    public static int rightX = 450, rightY = 30;
    public static int centerX = 0, centerY = 50;

    double avgRight = 0, avgCenter = 0;
    // red, not seeing the left line

    public static double NO_PROP = 85;



    public static int widthCenter = 400, heightCenter = 180;
    public static int widthRight = 190, heightRight = 200;

    public static int blueMinH = 200;
    public static int blueMinS = 50;
    public static int blueMinV = 25;
    public static int blueMaxH = 0;
    public static int blueMaxS = 0;
    public static int blueMaxV = 0;
    private Mat workingMatrix = new Mat();

    public enum Location
    {
        Left,
        Right,
        Center
    }

    Location location = Location.Center;


    public PropPipelineBlueRight()
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

        // Define regions of interest§
        Mat matCenter = workingMatrix.submat(centerY, heightCenter + centerY, centerX, centerX + widthCenter);
        Mat matRight = workingMatrix.submat(rightY, heightRight + rightY, rightX, rightX + widthRight);

        // Draw rectangles around regions of interest
        Imgproc.rectangle(workingMatrix, new Rect(centerX, centerY, widthCenter, heightCenter), new Scalar(0, 255, 0));
         Imgproc.rectangle(workingMatrix, new Rect(rightX, rightY, widthRight, heightRight), new Scalar(0, 255, 0));

        // Calculate the average intensity of blue color in each region
        avgCenter = Core.mean(matCenter).val[0];
        avgRight = Core.mean(matRight).val[0];


        // Find the region with the maximum average blue intensity
        if(avgRight > NO_PROP && avgCenter > NO_PROP)
        {
            location = Location.Left;
        }
        else if (avgRight > avgCenter) {
            location = Location.Center;
        } else if (avgCenter > avgRight) {
            location = Location.Right;
        }

        return workingMatrix;
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
}