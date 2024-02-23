package org.firstinspires.ftc.teamcode.auto.Actions.FailSafes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.testing.vision.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

public class CheckAprilTagAction implements Action {

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private Action mainAction, failOverAction;
    public CheckAprilTagAction(Action mainAction, Action failOverAction) {
        initCamera();

        this.mainAction = mainAction;
        this.failOverAction = failOverAction;

    }

    void initCamera() {
        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);


    }

    private boolean isAprilTagDetected()
    {
        return aprilTagDetectionPipeline.getLatestDetections().size() != 0;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (isAprilTagDetected() == false)
        {
            return this.failOverAction.run(telemetryPacket);
        }

        return mainAction.run(telemetryPacket);
    }
}

