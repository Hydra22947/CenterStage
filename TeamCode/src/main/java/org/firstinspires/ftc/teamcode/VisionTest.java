package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Vision Test", group = "Vision")
public class VisionTest extends LinearOpMode {
    public PropPipeline vision;
    Telemetry telemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive())
        {
            PropPipeline.Location teamPropLocation = vision.getLocation();
            telemetry.addData("Current prop location:", teamPropLocation.toString());
            telemetry.update();
        }
    }
}
