package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name="TEST Motor Test", group="tests")
@Config
public class TestMotorTest extends LinearOpMode {

    RobotHardware robotHardware = RobotHardware.getInstance();
    private DcMotor motor = null;

    public static double power = 0.1;

    @Override
    public void runOpMode() {
        robotHardware.init(hardwareMap, telemetry, true);


        waitForStart();

        while (opModeIsActive()) {
            robotHardware.extensionMotor.setPower(power);
        }
    }
}
