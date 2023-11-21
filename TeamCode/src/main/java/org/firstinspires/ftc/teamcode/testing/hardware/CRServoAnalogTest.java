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
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.jetbrains.annotations.NotNull;


@TeleOp(name="CRServoAnalog Test", group="tests")
@Config
public class CRServoAnalogTest extends LinearOpMode {
    public enum ExtensionState {
        OPEN,
        INTERMIDIATE,
        CLOSE,
        MANUAL
    }


    public static double OPEN_EXTENSION = 180;
    public static double OPEN_HALFWAY_EXTENSION = 100;// maybe for pixel at auto beginning
    public static double CLOSE_EXTENSION = 0.0;

    public static double MAX_POWER = 1;
    public static double kP = .1, kI = 0, kD = 0;
    private BetterGamepad cGamepad;
    private BasicPID pid;
    private PIDCoefficients pidCoefficients;
    private AngleController angleController;
    CRServo crServo;
    AnalogInput analogInput;

    ExtensionState state = ExtensionState.OPEN;

    public void updateState(@NotNull ExtensionState currentState) {
        switch (currentState) {
            case OPEN:
                setPosition(OPEN_EXTENSION);
                break;
            case INTERMIDIATE:
                setPosition(OPEN_HALFWAY_EXTENSION);
                break;
            case CLOSE:
                setPosition(CLOSE_EXTENSION);
                break;
            case MANUAL:
                crServo.setPower(Range.clip(-this.cGamepad.left_stick_y, -MAX_POWER, MAX_POWER));
                break;
        }
    }

    public void setPosition(double position) {
        double currentPosition = analogInput.getVoltage() / 3.3 * 360;
        crServo.setPower(this.angleController.calculate(Math.toRadians(position), Math.toRadians(currentPosition)));
        telemetry.addData("target", position);
        telemetry.addData("power", this.angleController.calculate(Math.toRadians(position), Math.toRadians(currentPosition)));
        telemetry.addData("pos", currentPosition);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        crServo  = hardwareMap.get(CRServo.class, "sT");
        analogInput = hardwareMap.get(AnalogInput.class, "aT");

        this.cGamepad = new BetterGamepad(gamepad1);
        this.pidCoefficients = new PIDCoefficients(kP, kI, kD);
        this.pid = new BasicPID(pidCoefficients);
        this.angleController = new AngleController(pid);


        waitForStart();

        while (opModeIsActive()) {
            cGamepad.update();
            if(cGamepad.A())
            {
                state = ExtensionState.OPEN;
            }
            else if(cGamepad.B())
            {
                state = ExtensionState.CLOSE;
            }
            else if(cGamepad.Y())
            {
                state = ExtensionState.MANUAL;
            }

            updateState(state);


            telemetry.update();
        }
    }
}
