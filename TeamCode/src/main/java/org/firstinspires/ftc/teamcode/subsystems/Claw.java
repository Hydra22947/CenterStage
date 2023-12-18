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
public class Claw
{
    public static double delay = 500;
    boolean shouldOpen = false;
    private final RobotHardware robot;
    public boolean overwrite = false;
    public enum ClawState
    {
        CLOSED,
        INTERMEDIATE,
        OPEN
    }

    public ClawState leftClaw = ClawState.OPEN;
    public ClawState rightClaw = ClawState.OPEN;

    public static double openLeft = 0.47, closeLeft = .56;
    public static double openRight = 0.57, closeRight = .65;
    double tempRight = closeRight;
    double tempLeft = closeLeft;
    CommandOpMode opMode;

    public Claw(CommandOpMode opMode)
    {
        this.opMode = opMode;
        this.robot = RobotHardware.getInstance();
        updateState(ClawState.OPEN, ClawSide.BOTH);
    }

    public void update() {
        if(shouldOpen)
        {
            updateState(ClawState.OPEN, ClawSide.BOTH);

            closeLeft = openLeft;
            closeRight = openRight;
        }
        else
        {
            closeRight = tempRight;
            closeLeft = tempLeft;
        }

        if((rightClaw == Claw.ClawState.OPEN || rightClaw == ClawState.CLOSED) && !shouldOpen && !overwrite)
        {
            checkAndClose(robot.breakbeamRight, ClawSide.RIGHT);
        }

        if((leftClaw == Claw.ClawState.OPEN || leftClaw == ClawState.CLOSED) && !shouldOpen && !overwrite)
        {
            checkAndClose(robot.breakbeamLeft, ClawSide.LEFT);
        }
    }

    public void updateState(@NotNull ClawState state, @NotNull ClawSide side) {
        double position = getClawStatePosition(state, side);

        switch(side) {
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

    private double getClawStatePosition(ClawState state, ClawSide side)
    {
        switch (side)
        {
            case LEFT:
                switch (state) {
                    case CLOSED:
                        return closeLeft;
                    case OPEN:
                        return openLeft;
                    default:
                        return 0.0;
                }
            case RIGHT:
                switch (state) {
                    case CLOSED:
                        return closeRight;
                    case OPEN:
                        return openRight;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }

    public boolean checkAndClose(DigitalChannel sensor, ClawSide side)
    {

        if(sensor.getState() && !shouldOpen)
        {
            updateState(ClawState.OPEN, side);
        }
       else if(!shouldOpen)
        {
            opMode.schedule(new SequentialCommandGroup(
                    new WaitCommand((long) delay),
                    new InstantCommand(() -> updateState(ClawState.CLOSED, side))));
        }

        return sensor.getState();
    }

    public boolean isShouldOpen() {
        return shouldOpen;
    }

    public void setShouldOpen(boolean shouldOpen) {
        this.shouldOpen = shouldOpen;
    }

    double getTime()
    {
        return System.nanoTime() / 1000000;
    }
}