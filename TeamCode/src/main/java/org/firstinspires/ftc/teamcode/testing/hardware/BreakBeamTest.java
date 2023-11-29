package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(name="Break Beam Test", group="tests")
@Config
public class BreakBeamTest extends LinearOpMode {

    private DigitalChannel beam = null;
    private DigitalChannel beam2 = null;

    public static String hw = "bbR";
    public static String hw2 = "bbL";

    @Override
    public void runOpMode() {
        beam  = hardwareMap.get(DigitalChannel.class, hw);
        beam2  = hardwareMap.get(DigitalChannel.class, hw2);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("stateR", beam.getState());
            telemetry.addData("stateL", beam2.getState());
            telemetry.update();
        }
    }
}
