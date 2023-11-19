package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Motor Intake Test", group="tests")
@Config
public class MotorTest extends LinearOpMode {

    private DcMotor motor = null;

    public static double power = 0.1;
    public static String hw = "mI";

    @Override
    public void runOpMode() {
        motor  = hardwareMap.get(DcMotor.class, hw);

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(power);
        }
    }
}
