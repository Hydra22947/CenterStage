package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class IntakeExtension extends BetterSubsystem {
    public enum ExtensionState {
        OPEN,
        INTERMEDIATE,
        CLOSE,
        MANUAL
    }

    public static double OPEN_EXTENSION = 180;
    public static double OPEN_HALFWAY_EXTENSION = 135;// maybe for pixel at auto beginning
    public static double CLOSE_EXTENSION = 90;
    public static double MAX_POWER = 1;

    public static double kP = 0.825, kI = 0, kD = 0.00051;

    private RobotHardware robot;
    private AbsoluteAnalogEncoder absoloutEncoder;
    private PIDFController.PIDCoefficients pidCoefficients;
    private PIDFController controller;
    private BetterGamepad cGamepad;

    ExtensionState current = ExtensionState.CLOSE;

    public IntakeExtension(Gamepad gamepad) {
        this.robot = RobotHardware.getInstance();
        this.absoloutEncoder = new AbsoluteAnalogEncoder(this.robot.extensionServoEncoder);

        this.cGamepad = new BetterGamepad(gamepad);
        this.pidCoefficients = new PIDFController.PIDCoefficients();
        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);

    }

    public void updateState() {
        cGamepad.update();

        switch (current) {
            case OPEN:
                setPosition(OPEN_EXTENSION, -MAX_POWER, MAX_POWER);
                break;
            case INTERMEDIATE:
                setPosition(OPEN_HALFWAY_EXTENSION, -MAX_POWER, MAX_POWER);
                break;
            case CLOSE:
                setPosition(CLOSE_EXTENSION, -MAX_POWER, MAX_POWER);
                break;
            case MANUAL:
                if(this.cGamepad.right_trigger != 0)
                {
                    setPosition(OPEN_EXTENSION, -this.cGamepad.right_trigger, this.cGamepad.right_trigger);
                }
                else
                {
                    setPosition(CLOSE_EXTENSION, -MAX_POWER, MAX_POWER);
                }
                break;
        }
    }

    public void setPosition(double position, double min, double max)
    {
        controller.updateError(Math.toRadians(position) - absoloutEncoder.getCurrentPosition());

        this.robot.extensionServo.setPower(Range.clip(controller.update(), min, max));
        robot.telemetry.addData("POWER", robot.extensionServo.getPower());
        robot.telemetry.addData("POWER PID", controller.update());
        robot.telemetry.addData("POS", absoloutEncoder.getCurrentPosition());
        robot.telemetry.update();
    }

    @Override
    public void periodic() {
        updateState();
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }

    public ExtensionState getCurrent() {
        return current;
    }

    public void setCurrent(ExtensionState current) {
        this.current = current;
    }
}
