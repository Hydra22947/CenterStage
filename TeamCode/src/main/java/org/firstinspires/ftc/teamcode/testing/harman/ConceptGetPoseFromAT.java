package org.firstinspires.ftc.teamcode.testing.harman;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name="ConceptGetPoseFromAT", group="testing")
public class ConceptGetPoseFromAT extends LinearOpMode {
    VisionPortal visionPortal;
    RobotHardware robot = RobotHardware.getInstance();
    AprilTagProcessor aprilTag;

    List<AprilTagDetection> detections;
    public final Vector2d[] tagPositions = new Vector2d[] {new Vector2d(62, 41.5), // id 1 left, blue
            new Vector2d(62, 35.5), // center, blue
            new Vector2d(62, 29.5), // right, blue
            new Vector2d(62,-29.5),
            new Vector2d(62,-35.5),
            new Vector2d(62, -41.5),
    };

    public static double x = 15.13, y = 3.4;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();

        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            detections = findDetections();
            reportDetections(detections);
            reportPose("Localizer pose: ", robot.drive.pose);
            reportPose("AprilTag pose: ", getPoseFromAprilTag());

            robot.drive.updatePoseEstimate();
            telemetry.update();
        }


    }

    public Pose2d getPoseFromAprilTag() {
        Vector2d deltaF = new Vector2d(x,y);

        List<AprilTagDetection> detections = findDetections();
        if (detections == null || detections.isEmpty()) {
            RobotLog.i("getPoseFromAprilTag: no detections");
            return robot.drive.pose;
        }
        AprilTagDetection OurTag = detections.get(0);
        for (AprilTagDetection d : detections) {
            if (OurTag.ftcPose == null) {
                OurTag = d;
                continue;
            }
            if (d.ftcPose == null) {
                continue;
            }
            if (Math.abs(d.ftcPose.x) < Math.abs(OurTag.ftcPose.x)) {
                OurTag = d;
            }
        }
        if (OurTag.ftcPose == null) {
            RobotLog.i("getPoseFromAprilTag: no detections");
            return robot.drive.pose;
        }

        Vector2d cameraVector = new Vector2d(OurTag.ftcPose.y, -OurTag.ftcPose.x);
        Vector2d rTag = tagPositions[OurTag.id - 1];
        Vector2d returnVector = rTag.minus(deltaF);
        returnVector = returnVector.minus(cameraVector);
        Pose2d returnPose = new Pose2d(returnVector, Math.toRadians(-OurTag.ftcPose.yaw));
        RobotLog.i("getPoseFromAprilTag: reference tag = "+OurTag.id);
        RobotLog.i(String.format("getPoseFromAprilTag: tag data: (%.3f, %.3f) @%.3f",OurTag.ftcPose.x, OurTag.ftcPose.y, OurTag.ftcPose.yaw));
        RobotLog.i("getPoseFromAprilTag: pose = "+returnPose.toString());

        return returnPose;

    }

    public List<AprilTagDetection> findDetections() {
        return aprilTag.getDetections();
    }

    public void reportDetections(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) {
            RobotLog.i("no detections seen");
            telemetry.addLine("no detections seen");
        }
        else {
            for (AprilTagDetection d : detections) {
                if(d.id == 3)
                {
                    String s;
                    if (d.ftcPose == null) {
                        s = String.format("Tag ID %d, no pose available", d.id);
                    }
                    else {
                        s = String.format("Tag ID %d: (%.3f, %.3f) @%.3f", d.id, d.ftcPose.y, -d.ftcPose.x, d.ftcPose.yaw);
                    }
                    RobotLog.i(s);
                    telemetry.addLine(s);
                }
            }

        }
    }

    public void reportPose(String tag, Pose2d pose) {
        tag = tag + String.format("(%.3f, %.3f) @ %.3f", pose.position.x, pose.position.y, pose.heading.toDouble());
        RobotLog.i(tag);
        telemetry.addLine(tag);
    }

}