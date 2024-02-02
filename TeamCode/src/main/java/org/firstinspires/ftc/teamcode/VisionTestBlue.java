package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Vision Test Blue", group = "Vision")
public class VisionTestBlue extends LinearOpMode {
    public PropPipelineRedRightBlueLeft vision;
    private VisionPortal portal;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        vision = new PropPipelineRedRightBlueLeft(telemetry, PropPipelineRedRightBlueLeft.Alliance.BLUE);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessors(vision)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(vision, 0);


        waitForStart();
        while (opModeIsActive())
        {
            PropPipelineRedRightBlueLeft.Location teamPropLocation = vision.getLocation();

            telemetry.addData("Current prop location:", teamPropLocation.toString());
            telemetry.update();
        }
    }
}
