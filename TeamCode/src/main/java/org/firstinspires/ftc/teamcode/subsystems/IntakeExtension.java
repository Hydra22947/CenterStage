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

    public static double OPEN_EXTENSION = 0.5;
    public static double OPEN_HALFWAY_EXTENSION = OPEN_EXTENSION / 2;// maybe for pixel at auto beginning
    public static double CLOSE_EXTENSION = 0;

    private RobotHardware robot;
    private BetterGamepad cGamepad;

    public static ExtensionState current = ExtensionState.CLOSE;

    public IntakeExtension(Gamepad gamepad) {
        this.robot = RobotHardware.getInstance();

        this.cGamepad = new BetterGamepad(gamepad);

    }

    public void updateState() {
        cGamepad.update();

        switch (current) {
            case OPEN:
                this.robot.extensionServo.setPosition(OPEN_EXTENSION);
                break;
            case INTERMEDIATE:
                this.robot.extensionServo.setPosition(OPEN_HALFWAY_EXTENSION);
                break;
            case CLOSE:
                this.robot.extensionServo.setPosition(CLOSE_EXTENSION);
                break;
            case MANUAL:
                this.robot.extensionServo.setPosition(Range.clip(cGamepad.right_trigger, CLOSE_EXTENSION, OPEN_EXTENSION));
                break;
        }
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
