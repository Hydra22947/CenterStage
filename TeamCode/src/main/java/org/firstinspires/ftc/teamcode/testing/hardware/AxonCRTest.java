package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;


@TeleOp(name="CR Axon Test", group="tests")
@Config
@Disabled
public class AxonCRTest extends LinearOpMode {

    CRServo crServo;
    AnalogInput analogInput, analogInput2;
    AbsoluteAnalogEncoder absoluteAnalogEncoder, absoluteAnalogEncoder2;
    double power1 = 0;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        crServo  = hardwareMap.get(CRServo.class, "s1");
        analogInput = hardwareMap.get(AnalogInput.class, "a1");
        absoluteAnalogEncoder = new AbsoluteAnalogEncoder(analogInput);
        absoluteAnalogEncoder2 = new AbsoluteAnalogEncoder(analogInput2);


        waitForStart();

        while (opModeIsActive()) {
            crServo.setPower(0);


            telemetry.addData("1 pos", Math.toDegrees(absoluteAnalogEncoder.getCurrentPosition()));
            telemetry.addData("1 power", power1);

            telemetry.update();
        }
    }
}
