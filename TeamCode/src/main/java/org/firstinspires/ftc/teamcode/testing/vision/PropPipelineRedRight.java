package org.firstinspires.ftc.teamcode.testing.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.android.Utils;
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
    public static int rightX = 450, rightY = 40;
    public static int centerX = 0, centerY = 50;

    double avgRight = 0, avgCenter = 0;
    // red, not seeing the left line

    public static double NO_PROP_CENTER = 100;
    public static double NO_PROP_RIGHT = 100;


    public static int widthRight = 175, heightRight = 190;
    public static int widthCenter = 350, heightCenter = 180;

    public static int redMinH = 115;
    public static int redMinS = 0;
    public static int redMinV = 0;
    public static int redMaxH = 130;
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
        workingMatrix.copyTo(returnMatrix);
        Core.bitwise_and(input, input, workingMatrix);

        // TODO: remove bitwise, and get returnMatrix and add the left center and that i will count the white
        // Define regions of interest
        Mat matLeft = workingMatrix.submat(rightY, heightRight + rightY, rightX, rightX + widthRight);
        Mat matCenter = workingMatrix.submat(centerY, heightCenter + centerY, centerX, centerX + widthCenter);

        // Draw rectangles around regions of interest
        Imgproc.rectangle(workingMatrix, new Rect(rightX, rightY, widthRight, heightRight), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(centerX, centerY, widthCenter, heightCenter), new Scalar(0, 255, 0));

        // Calculate the average intensity of blue color in each region
        avgRight = Core.mean(matLeft).val[2];
        avgCenter = Core.mean(matCenter).val[2];


        // Find the region with the maximum average blue intensity
        if(avgRight > NO_PROP_RIGHT && avgCenter > NO_PROP_CENTER)
        {
            location = Location.Left;
        }
        else if (avgRight < NO_PROP_RIGHT) {
            location = Location.Center;
        } else if (avgCenter < NO_PROP_CENTER) {
            location = Location.Right;
        }

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

    public static void setNoProp(double noProp) {
        NO_PROP_CENTER = noProp;
    }

    public static double getNoProp() {
        return NO_PROP_CENTER;
    }
}