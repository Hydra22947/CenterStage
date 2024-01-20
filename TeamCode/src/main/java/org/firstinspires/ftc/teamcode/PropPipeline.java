package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class PropPipeline implements VisionProcessor {
    public enum Location {
        LEFT, CENTER, RIGHT, BLUE, RED, FAR, CLOSE
    }
    private static final boolean DEBUG = false;

    private volatile Location location = Location.RIGHT;

    private final Mat hsv = new Mat();

    public static int redLeftX = (int) (815 / 1.5);
    public static int redLeftY = (int) (550 / 1.5);

    public static int redCenterX = (int) (1365 / 1.5);
    public static int redCenterY = (int) (475 / 1.5);

    public static int blueLeftX = (int) (240 / 1.5);
    public static int blueLeftY = (int) (525 / 1.5);

    public static int blueCenterX = (int) (925 / 1.5);
    public static int blueCenterY = (int) (485 / 1.5);

    public static int leftWidth = (int) (175 / 1.5);
    public static int leftHeight = (int) (100 / 1.5);

    public static int centerWidth = (int) (125 / 1.5);
    public static int centerHeight = (int) (125 / 1.5);

    public static double BLUE_TRESHOLD = 70;
    public static double RED_TRESHOLD = 100;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    public Scalar left = new Scalar(0, 0, 0);
    public Scalar center = new Scalar(0, 0, 0);

    Telemetry telemetry;

//    Location ALLIANCE = Location.RED;

    public PropPipeline() {
        this(null);
    }

    public PropPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Rect leftZoneArea;
        Rect centerZoneArea;
        //TODO:ADD FOR BOTH SIDES.
        leftZoneArea = new Rect(redLeftX, redLeftY, leftWidth, leftHeight);
        centerZoneArea = new Rect(redCenterX, redCenterY, centerWidth, centerHeight);


        Mat leftZone = frame.submat(leftZoneArea);
        Mat centerZone = frame.submat(centerZoneArea);


        if (DEBUG) {
            Imgproc.blur(frame, frame, new Size(5, 5));
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 2);
            Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 2);
        }

        Imgproc.blur(leftZone, leftZone, new Size(5, 5));
        Imgproc.blur(centerZone, centerZone, new Size(5, 5));

        left = Core.mean(leftZone);
        center = Core.mean(centerZone);

        if (telemetry != null) {
            telemetry.addData("leftColor", left.toString());
            telemetry.addData("centerColor", center.toString());
            telemetry.addData("analysis", location.toString());
            telemetry.update();
        }

        //TODO:CHANGE TO MATCH BOTH SIDES
        double threshold = RED_TRESHOLD;
        int idx = 0 ;

        leftColor = left.val[idx];
        centerColor = center.val[idx];

        if (leftColor > threshold && (left.val[0] + left.val[1] + left.val[2] - left.val[idx] < left.val[idx])) {
            // left zone has it
            location = Location.LEFT;
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 10);
        } else if (centerColor > threshold && (center.val[0] + center.val[1] + center.val[2] - center.val[idx] < center.val[idx])) {
            // center zone has it
            location = Location.CENTER;
            Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 10);
        } else {
            // right zone has it
            location = Location.RIGHT;
        }

        leftZone.release();
        centerZone.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Location getLocation() {
        return this.location;
    }
}
