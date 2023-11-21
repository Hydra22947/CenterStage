package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
// TODO: needs to be CRServo with position limits
public class IntakeExtension extends BetterSubsystem {
    public enum ExtensionState {
        OPEN,
        INTEMIDIATE,
        CLOSE
    }

    public static double OPEN_EXTENSION = 0.4;
    public static double OPEN_HALFWAY_EXTENSION = 0.5;// maybe for pixel at auto beginning
    public static double CLOSE_EXTENSION = 0.0;

    private RobotHardware robot;
    ExtensionState current = ExtensionState.CLOSE;

    public IntakeExtension() {
        this.robot = RobotHardware.getInstance();
    }

    public void updateState(@NotNull ExtensionState currentState) {
        switch (currentState) {
            case OPEN:
                setPosition(OPEN_EXTENSION);
                break;
            case INTEMIDIATE:
                setPosition(OPEN_HALFWAY_EXTENSION);
                break;
            case CLOSE:
                setPosition(CLOSE_EXTENSION);
                break;
        }
        current = currentState;
    }

    public void setPosition(double position) {
        this.robot.extensionServo.setPosition(position);
    }

    @Override
    public void periodic() {
        updateState(current);
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
}
