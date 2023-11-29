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
    private Servo servo2 = null;
    private Servo servo3 = null;
    public static String hw = "sCR";
    public static String hw2 = "sCL";
    public static String hw3 = "sC";

    public static double pos1 = 0.35;
    public static double pos2 = 0.35;
    public static double pos3 = 0.5;

    public static boolean use1 = true;
    public static boolean use2 = true;
    public static boolean use3 = false;

    @Override
    public void runOpMode() {
        if(use1)
            servo1  = hardwareMap.get(Servo.class, hw);

        if(use2)
            servo2  = hardwareMap.get(Servo.class, hw2);

        if(use3)
            servo3  = hardwareMap.get(Servo.class, hw3);

        waitForStart();

        while (opModeIsActive()) {
            if(use1)
                servo1.setPosition(pos1);

            if(use2)
                servo2.setPosition(pos2);

            if(use3)
                servo3.setPosition(pos3);
        }
    }
}
