package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="CRServo Test", group="tests")
@Config
public class CRServoTest extends LinearOpMode {

    private CRServo intake = null;

    public static double power = 0.1;

    @Override
    public void runOpMode() {
        intake  = hardwareMap.get(CRServo.class, "sI");

        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(power);
        }    }
}
