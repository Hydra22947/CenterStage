package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Claw Servo Test", group="tests")
@Config
public class ClawServoTest extends LinearOpMode {

    private Servo servo1 = null;
    private Servo servo2 = null;
    public static String hw = "s1";
    public static String hw2 = "s2";

    public static double pos1 = 0;
    public static double pos2 = 0;

    @Override
    public void runOpMode() {
        servo1  = hardwareMap.get(Servo.class, hw);
        servo2 = hardwareMap.get(Servo.class, hw2);

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(pos1);
            servo2.setPosition(pos2);
        }
    }
}
