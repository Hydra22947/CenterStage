package org.firstinspires.ftc.teamcode.testing.hardware;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.jetbrains.annotations.NotNull;


@TeleOp(name="CRServoAnalog Test", group="tests")
@Config
public class CRServoAnalogTest extends LinearOpMode {

    public static double target = 0;

    public static double MAX_POWER = 1;
    public static double kP = 0.525, kI = 0, kD = 0.00051;
    private BetterGamepad cGamepad;
    private PIDFController.PIDCoefficients pidCoefficients;
    PIDFController controller;
    CRServo crServo;
    AnalogInput analogInput;
    AbsoluteAnalogEncoder absoluteAnalogEncoder;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        crServo  = hardwareMap.get(CRServo.class, "sT");
        analogInput = hardwareMap.get(AnalogInput.class, "aT");
        absoluteAnalogEncoder = new AbsoluteAnalogEncoder(analogInput);

        this.cGamepad = new BetterGamepad(gamepad1);
        this.pidCoefficients = new PIDFController.PIDCoefficients();
        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);

        waitForStart();

        while (opModeIsActive()) {
            cGamepad.update();

            double currentPosition = analogInput.getVoltage() / 3.3 * 360;

            controller.targetPosition = Math.toRadians(target);

            telemetry.addData("target", target);
            telemetry.addData("power", controller.update(Math.toRadians(currentPosition)));
            telemetry.addData("pos", currentPosition);
            telemetry.addData("pos2", absoluteAnalogEncoder.getCurrentPosition());
            telemetry.update();

            if(gamepad1.left_stick_y != 0)
            {
                crServo.setPower(Range.clip(-this.cGamepad.left_stick_y, -MAX_POWER, MAX_POWER));
            }
            else
            {
                crServo.setPower(controller.update(absoluteAnalogEncoder.getCurrentPosition()));
            }

            telemetry.update();
        }
    }
}
