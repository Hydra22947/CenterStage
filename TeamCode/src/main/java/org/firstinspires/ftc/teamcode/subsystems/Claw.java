package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.OpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.jetbrains.annotations.NotNull;

@Config
public class Claw implements Subsystem {
    private final RobotHardware robot;

    @Override
    public void play() {

    }

    @Override
    public void loop(boolean allowMotors) {
        updateState(leftClaw, ClawSide.LEFT);
        updateState(rightClaw, ClawSide.RIGHT);
    }

    @Override
    public void stop() {

    }

    public enum ClawState {
        CLOSED,
        INTERMEDIATE,
        INTAKE,
        OPEN
    }

    public ClawState leftClaw = ClawState.OPEN;
    public ClawState rightClaw = ClawState.OPEN;

    // LOOK FORM INTAKE
    public static double openLeft = .17, closeLeft = .3;
    public static double intakeRight = .45, intakeLeft = .2;
    public static double openRight = .42, closeRight = .53;
    public static double intermediateLeft = .25, intermediateRight = .48;

    public Claw() {
        this.robot = RobotHardware.getInstance();
        updateState(ClawState.OPEN, ClawSide.BOTH);
    }

    public void update() {
        updateState(leftClaw, ClawSide.LEFT);
        updateState(rightClaw, ClawSide.RIGHT);
    }

    public void updateState(@NotNull ClawState state, @NotNull ClawSide side) {
        double position = getClawStatePosition(state, side);

        switch (side) {
            case LEFT:
                robot.outtakeClawLeftServo.setPosition(position);
                this.leftClaw = state;
                break;
            case RIGHT:
                robot.outtakeClawRightServo.setPosition(position);
                this.rightClaw = state;
                break;
            case BOTH:
                position = getClawStatePosition(state, ClawSide.LEFT);
                robot.outtakeClawLeftServo.setPosition(position);
                this.leftClaw = state;
                position = getClawStatePosition(state, ClawSide.RIGHT);
                robot.outtakeClawRightServo.setPosition(position);
                this.rightClaw = state;
                break;
        }


    }

    private double getClawStatePosition(ClawState state, ClawSide side) {
        switch (side) {
            case LEFT:
                switch (state) {
                    case CLOSED:
                        return closeLeft;
                    case INTERMEDIATE:
                        return intermediateLeft;
                    case INTAKE:
                        return intakeLeft;
                    case OPEN:
                        return openLeft;
                    default:
                        return 0.0;
                }
            case RIGHT:
                switch (state) {
                    case CLOSED:
                        return closeRight;
                    case INTERMEDIATE:
                        return intermediateRight;
                    case INTAKE:
                        return intakeRight;
                    case OPEN:
                        return openRight;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }


    public void setLeftClaw(ClawState leftClaw) {
        this.leftClaw = leftClaw;
    }

    public void setRightClaw(ClawState rightClaw) {
        this.rightClaw = rightClaw;
    }

    public void setBothClaw(ClawState state) {
        this.rightClaw = state;
        this.leftClaw = state;
    }
}