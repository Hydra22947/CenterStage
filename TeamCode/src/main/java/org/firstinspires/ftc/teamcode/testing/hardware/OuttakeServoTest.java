package org.firstinspires.ftc.teamcode.testing.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;


@TeleOp(name="Outtake Servo Test", group="tests")
@Config
public class OuttakeServoTest extends LinearOpMode {

    public static double target = 0;

    private BetterGamepad cGamepad;
    Servo servo, servo2;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        servo  = hardwareMap.get(Servo.class, "s3");
        servo2  = hardwareMap.get(Servo.class, "s4");

        this.cGamepad = new BetterGamepad(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            cGamepad.update();

            servo.setPosition(target);
            servo2.setPosition(target);

            telemetry.update();
        }
    }
}
