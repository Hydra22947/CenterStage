package org.firstinspires.ftc.teamcode.testing;

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

    public static double pos1 = 0;
    public static double pos2 = 0;

    @Override
    public void runOpMode() {
        servo1  = hardwareMap.get(Servo.class, "sHR");
        servo1.setDirection(Servo.Direction.REVERSE);
        servo2  = hardwareMap.get(Servo.class, "sHL");
        servo3  = hardwareMap.get(Servo.class, "sC");
        servo3.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(pos1);
            servo2.setPosition(pos1);
            servo3.setPosition(pos2);
        }
    }
}
