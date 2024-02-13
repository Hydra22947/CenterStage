package org.firstinspires.ftc.teamcode.auto.rr1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.AutoSettings;
import org.firstinspires.ftc.teamcode.auto.rr1.commands.InitSystems;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public abstract class HydraAutoBase extends CommandOpMode {
    protected Pose2d startPose;
    AutoSettings.AllianceColor allianceColor;
    AutoSettings.AllianceSide allianceSide;
//    protected PropDetector.DetectionResult detectionResult = PropDetector.DetectionResult.LEFT;
    public RobotHardware robot = RobotHardware.getInstance();
    protected Elevator elevator;
    protected Intake intake;
    protected Outtake outtake;
    protected Claw claw;
    protected IntakeExtension intakeExtension;
//    public WebcamName webcam1;

//    protected PropDetectionProcessor propDetectionProcessor;
//    protected AprilTagProcessor aprilTagProcessor;
//    public VisionPortal vision;

    public HydraAutoBase(AutoSettings.AllianceColor allianceColor, AutoSettings.AllianceSide allianceSide, Pose2d startPose) {
        this.allianceColor = allianceColor;
        this.allianceSide = allianceSide;
        this.startPose = startPose;
    }

//    public void localizeWithRawAprilTag(boolean isBackCamera, AprilTagDetection detection) {
//        Pose2d robotPos = AprilTagLocalization.getRobotPositionFromTag(detection, robot.m_drive.pose.heading.toDouble(), isBackCamera);
//        this.robot.m_drive.pose = robotPos;
//    }

//    public ArrayList<AprilTagDetection> getAprilTagDetections(boolean isBackCamera) {
//        if (!vision.getProcessorEnabled(aprilTagProcessor)) {
//            vision.setProcessorEnabled(aprilTagProcessor, true);
//        }
//
//        return aprilTagProcessor.getDetections();
//    }

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, telemetry, startPose);

        elevator = new Elevator(true);
        outtake = new Outtake();
        claw = new Claw();
        intake = new Intake();
        intakeExtension = new IntakeExtension(true);

        intakeExtension.setAuto(true);
        elevator.setAuto(true);

        new InitSystems(elevator, outtake, claw, intake, intakeExtension).schedule();

//        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

//        propDetectionProcessor = new PropDetectionProcessor(m_allianceType, m_allianceSide);
//        aprilTagProcessor = new AprilTagProcessor.Builder().build();
//        vision = new VisionPortal.Builder()
//                .setCamera(switchableCamera)
//                .setCameraResolution(new Size(640, 480))
//                .enableLiveView(true)
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .addProcessors(aprilTagProcessor)
//                .build();
//
//        while (vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addLine("Waiting for cameras to be ready...");
//            telemetry.update();
//            sleep(10);
//        }
//
//        vision.setActiveCamera(webcam1);
//        vision.setProcessorEnabled(aprilTagProcessor, false);

        initAuto();

        while (opModeInInit()) {
//            telemetry.addData("Detected", detectionResult);
//            telemetry.update();
//            sleep(25);
        }

//        vision.setActiveCamera(webcam2);
//        vision.setProcessorEnabled(aprilTagProcessor, true);
//        vision.setProcessorEnabled(propDetectionProcessor, false);

        startAuto();
    }

    @Override
    public void run() {
        super.run();
        robot.drive.updatePoseEstimate();
    }

    public abstract void initAuto();
    public abstract void startAuto();
}