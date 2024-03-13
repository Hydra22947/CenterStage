package org.firstinspires.ftc.teamcode.auto.Actions.FailSafes;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.testing.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

public class CheckAprilTagAction implements Action {

    RobotHardware robot = RobotHardware.getInstance();
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    private Action mainAction, continueAction;

    public CheckAprilTagAction(Action mainAction, Action continueAction) {
        initCamera();

        this.mainAction = mainAction;
        this.continueAction = continueAction;
    }

    void initCamera() {


        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        aprilTag.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public void reportDetections(List<AprilTagDetection> detections, TelemetryPacket telemetryPacket) {
        if (detections == null || detections.isEmpty()) {
            RobotLog.i("no detections seen");
            telemetryPacket.addLine("no detections seen");
        }
        else {
            for (AprilTagDetection d : detections) {
                String s = String.format("Tag ID %d, no pose available", d.id);

                RobotLog.i(s);
                telemetryPacket.addLine(s);
            }

        }
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (aprilTag.getDetections().size() > 2)
        {
            return this.continueAction.run(telemetryPacket);
        }

        reportDetections(aprilTag.getDetections(), telemetryPacket);

        return mainAction.run(telemetryPacket);
    }
}

