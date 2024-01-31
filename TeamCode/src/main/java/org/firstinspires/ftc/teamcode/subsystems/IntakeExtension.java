package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class IntakeExtension implements Subsystem{
    @Override
    public void play() {

    }

    @Override
    public void loop(boolean allowMotors) {
        update();
    }

    @Override
    public void stop() {

    }

    public enum ExtensionState {
        OPEN,
        CLOSE,
        MANUAL
    }

    public static double OPEN_EXTENSION_R = 0.62, HALF_OPEN_R = .52;
    public static double OPEN_EXTENSION_L = 0.6, HALF_OPEN_L = .5;

    public static double AUTO_OPEN_EXTENSION_R = 0.58;
    public static double AUTO_OPEN_EXTENSION_L = 0.48;
    public static double CLOSE_EXTENSION_R = 0.17;
    public static double CLOSE_EXTENSION_L = 0.15;

    private RobotHardware robot;
    private BetterGamepad cGamepad;

    public static ExtensionState current = ExtensionState.CLOSE;

    public IntakeExtension(Gamepad gamepad) {
        this.robot = RobotHardware.getInstance();

        this.cGamepad = new BetterGamepad(gamepad);

    }

    public IntakeExtension() {
        this.robot = RobotHardware.getInstance();
        cGamepad = null;
    }

    public void update() {
        if(cGamepad != null)
        {
            cGamepad.update();
        }

        switch (current) {
            case OPEN:
                this.robot.extensionServoRight.setPosition(OPEN_EXTENSION_R);
                this.robot.extensionServoLeft.setPosition(OPEN_EXTENSION_L);
                break;
            case CLOSE:
                this.robot.extensionServoRight.setPosition(CLOSE_EXTENSION_R);
                this.robot.extensionServoLeft.setPosition(CLOSE_EXTENSION_L);
                break;
            case MANUAL:
                this.robot.extensionServoRight.setPosition(Range.clip(cGamepad.right_trigger, CLOSE_EXTENSION_R, OPEN_EXTENSION_R));
                this.robot.extensionServoLeft.setPosition(Range.clip(cGamepad.right_trigger, CLOSE_EXTENSION_L, OPEN_EXTENSION_L));
                break;
        }
    }

    public ExtensionState getCurrent() {
        return current;
    }

    public void setCurrent(ExtensionState current) {
        this.current = current;
    }

    public void openExtension()
    {
        this.robot.extensionServoRight.setPosition(OPEN_EXTENSION_R);
        this.robot.extensionServoLeft.setPosition(OPEN_EXTENSION_L);
    }

    public void openExtensionAuto()
    {
        this.robot.extensionServoRight.setPosition(AUTO_OPEN_EXTENSION_R);
        this.robot.extensionServoLeft.setPosition(AUTO_OPEN_EXTENSION_L);
    }
    public void halfOpenExtension()
    {
        this.robot.extensionServoRight.setPosition(HALF_OPEN_R);
        this.robot.extensionServoLeft.setPosition(HALF_OPEN_L);
    }
    public void closeExtension()
    {
        this.robot.extensionServoRight.setPosition(CLOSE_EXTENSION_R);
        this.robot.extensionServoLeft.setPosition(CLOSE_EXTENSION_L);
    }
}
