package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Servo Test", group="tests")
@Config
public class ServoTest extends LinearOpMode {

    private Servo servo1 = null;
    public static String hw = "sIHPR";

    public static double pos1 = 0;

    @Override
    public void runOpMode() {
        servo1  = hardwareMap.get(Servo.class, hw);

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(pos1);
        }
    }
}
