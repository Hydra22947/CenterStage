package org.firstinspires.ftc.teamcode.testing.vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class PlainPipeline extends OpenCvPipeline {

    // blue , not seeing the right line


    public PlainPipeline()
    {
    }

    @Override
    public final Mat processFrame(Mat input) {
        return input;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


}