package org.firstinspires.ftc.teamcode.testing.hardware;

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

    private CRServo servo = null;

    public static double power = 0.1;
    public static String hw = "sI";

    @Override
    public void runOpMode() {
        servo  = hardwareMap.get(CRServo.class, hw);

        waitForStart();

        while (opModeIsActive()) {
            servo.setPower(power);
        }
    }
}
