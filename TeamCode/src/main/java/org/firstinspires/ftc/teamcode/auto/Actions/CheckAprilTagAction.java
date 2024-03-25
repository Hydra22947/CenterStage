//package org.firstinspires.ftc.teamcode.auto.Actions;
//
//import android.util.Size;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.RobotLog;
//
//import org.checkerframework.checker.units.qual.C;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.testing.vision.AprilTagDetectionPipeline;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.openftc.easyopencv.OpenCvCamera;
//
//import java.util.List;
//import java.util.Timer;
//
//@Config
//public class CheckAprilTagAction implements Action {
//    private Action mainAction, stopAction;
//    boolean found = false;
//    static boolean detect = false;
//    static double detectTime = 0;
//    public double detectDelay = 1;
//    public static double timeLeftForAuto = 3;
//    ElapsedTime time;
//    boolean shouldUseAprilTag;
//
//    public CheckAprilTagAction(AprilTagProcessor aprilTag, VisionPortal visionPortal, boolean shouldUseAprilTag, ElapsedTime time, Action mainAction, Action stopAction) {
//        this.time = time;
//        detect = false;
//
//        this.mainAction = mainAction;
//        this.stopAction = stopAction;
//
//        this.shouldUseAprilTag = shouldUseAprilTag;
//    }
//
//
//    public static void initAprilTag(AprilTagProcessor aprilTag, VisionPortal visionPortal, HardwareMap hardwareMap) {
//
//
//        aprilTag = new AprilTagProcessor.Builder()
//                .setDrawAxes(false)
//                .setDrawCubeProjection(false)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//                .build();
//
//        aprilTag.setDecimation(1);
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessors(aprilTag)
//                .setLiveViewContainerId(0)
//                .setCameraResolution(new Size(640, 480))
//                .build();
//
//    }
//
//    public void reportDetections(List<AprilTagDetection> detections, TelemetryPacket telemetryPacket) {
//        if (detections == null || detections.isEmpty()) {
//            RobotLog.i("no detections seen");
//            telemetryPacket.addLine("no detections seen");
//        }
//        else {
//            for (AprilTagDetection d : detections) {
//                String s = String.format("Tag ID %d, no pose available", d.id);
//
//                RobotLog.i(s);
//                telemetryPacket.addLine(s);
//            }
//
//        }
//    }
//
//    public static class StopAction implements Action {
//
//        RobotHardware robotHardware = RobotHardware.getInstance();
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            robotHardware.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
//            return true;
//        }
//
//    }
//
//    public static Action waitAction() {
//        return new StopAction();
//    }
//
//    public static void turnOnDetection()
//    {
//        detect = true;
//        detectTime = System.currentTimeMillis();
//    }
//
//
//    @Override
//    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//        if(shouldUseAprilTag && (detect && (((System.currentTimeMillis() - detectTime) / 1000) >= detectDelay) && aprilTag.getDetections().size() > 1) || found)
//        {
//            found = true;
//            return this.mainAction.run(telemetryPacket);
//        }
//        else if(shouldUseAprilTag && !found && (detect && (((System.currentTimeMillis() - detectTime) / 1000) >= detectDelay)) && ((30 - time.seconds()) >= timeLeftForAuto))
//        {
//            return stopAction.run(telemetryPacket);
//
//        }
//        reportDetections(aprilTag.getDetections(), telemetryPacket);
//        return mainAction.run(telemetryPacket);
//    }
//
//
//}
//
