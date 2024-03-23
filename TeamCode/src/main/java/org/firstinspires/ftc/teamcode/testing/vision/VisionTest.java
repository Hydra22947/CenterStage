package org.firstinspires.ftc.teamcode.testing.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name="Vision Test", group = "A")
public class VisionTest extends LinearOpMode{


    PropPipelineBlueLeft propPipelineBlueLeft;
    PropPipelineBlueRight propPipelineBlueRight;
    PropPipelineRedLeft propPipelineRedLeft;
    PropPipelineRedRight propPipelineRedRight;
    OpenCvWebcam webcam;
    BetterGamepad gamepad;

    enum SideColor
    {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    public static SideColor sideColor = SideColor.RED_RIGHT;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new BetterGamepad(gamepad1);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        propPipelineBlueLeft = new PropPipelineBlueLeft();
        propPipelineBlueRight = new PropPipelineBlueRight();
        propPipelineRedLeft = new PropPipelineRedLeft();
        propPipelineRedRight = new PropPipelineRedRight();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        switch (sideColor)
        {
            case BLUE_LEFT:
                webcam.setPipeline(propPipelineBlueLeft);
                break;
            case BLUE_RIGHT:
                webcam.setPipeline(propPipelineBlueRight);
                break;
            case RED_LEFT:
                webcam.setPipeline(propPipelineRedLeft);
                break;
            case RED_RIGHT:
                webcam.setPipeline(propPipelineRedRight);
                break;
        }

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);


        telemetry.addLine("Waiting for start");
        telemetry.update();

        while (opModeInInit())
        {
            gamepad.update();

            switch (sideColor)
            {
                case BLUE_LEFT:
                    telemetry.addData("POS", propPipelineBlueLeft.location.toString());
                    break;
                case BLUE_RIGHT:
                    telemetry.addData("POS", propPipelineBlueRight.location.toString());
                    break;
                case RED_LEFT:
                    telemetry.addData("POS", propPipelineRedLeft.location.toString());
                    break;
                case RED_RIGHT:
                    telemetry.addData("POS", propPipelineRedRight.location.toString());
                    break;
            }

            telemetry.update();
        }

        waitForStart();

        webcam.stopStreaming();

    }


}