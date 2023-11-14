package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Motor Intake Test", group="tests")
@Config
public class MotorIntakeTest extends LinearOpMode {

    private DcMotor intake = null;

    public static double power = 0.1;

    @Override
    public void runOpMode() {
        intake  = hardwareMap.get(DcMotor.class, "mI");

        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(power);
        }
    }
}
