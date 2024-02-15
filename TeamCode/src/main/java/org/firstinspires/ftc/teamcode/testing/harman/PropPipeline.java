package org.firstinspires.ftc.teamcode.testing.harman;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.auto.AutoSettings;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class PropPipeline extends OpenCvPipeline {

    // blue , not seeing the right line
    public static int leftX = 0, leftY = 55;
    public static int centerX = 280, centerY = 20;


    // red, not seeing the left line
//    public static int centerX = 0, centerY = 20;
    public static int rightX = 450, rightY = 30;


    //TODO: measure far

    public static int widthLeft = 175, heightLeft = 250;
    public static int widthCenter = 350, heightCenter = 200;
    public static int widthRight = 200, heightRight = 200;

    private Mat workingMatrix = new Mat();
    enum Location
    {
        Left,
        Right,
        Center
    }

    Location location = Location.Center;

    AutoSettings.AllianceColor allianceColor;
    AutoSettings.AllianceSide allianceSide;

    public PropPipeline(AutoSettings.AllianceColor allianceColor, AutoSettings.AllianceSide allianceSide)
    {
        this.allianceColor = allianceColor;
        this.allianceSide = allianceSide;
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if(workingMatrix.empty())
        {
            return input;
        }

        if(allianceColor == AutoSettings.AllianceColor.BLUE && allianceSide == AutoSettings.AllianceSide.CLOSE)
        {

        }

        Mat matLeft = workingMatrix.submat(120, 150, 10, 50);
        Mat matCenter = workingMatrix.submat(120, 150, 80, 120);
        Mat matRight = workingMatrix.submat(120, 150, 150, 190);

        Imgproc.rectangle(workingMatrix, new Rect(leftX, leftY, widthLeft, heightLeft), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(centerX, centerY, widthCenter, heightCenter), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(rightX, rightY, widthRight, heightRight), new Scalar(0, 255, 0));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];

        if(leftTotal > centerTotal)
        {
            if(leftTotal >rightTotal)
            {
                location = Location.Left;
            }
            else
            {
                location = Location.Right;
            }
        }
        else
        {
            if(centerTotal > rightTotal)
            {
                location = Location.Center;
            }
            else
            {
                location = Location.Right;
            }
        }

        return workingMatrix;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Location getLocation() {
        return location;
    }
}