package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.jetbrains.annotations.NotNull;

@Config
public class OuttakeSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    public static double intakeHandPivot = 0.75, intakeClawPivot = 0.095;
    public static double outtakeHandPivot = 0.29, outtakeClawPivot = 0.935;
    public static double outtakeHandPivotLong = .41, outtakeClawPivotLong = .89;

    public ClawState leftClaw = ClawState.OPEN;
    public ClawState rightClaw = ClawState.OPEN;

    // LOOK FORM INTAKE
    public static double openLeft = .36, closeLeft = 0.48;
    public static double intakeRight = .4, intakeLeft = .38; // misha
    public static double openRight = .38, closeRight = 0.49;
    public static double intermediateLeft = .41, intermediateRight = .44;


    public enum ClawState {
        CLOSED,
        INTERMEDIATE,
        INTAKE,
        OPEN,

    }

    public enum Angle
    {
        INTAKE,
        OUTTAKE,
        OUTTAKE_LONG
    }

    public enum Type
    {
        CLAW,
        HAND
    }

    Angle angle = Angle.INTAKE;
    public OuttakeSubsystem()
    {
        this.robot = RobotHardware.getInstance();
    }


    @Override
    public void periodic() {
        updateState(Type.HAND);
        updateState(Type.CLAW);
        updateState(leftClaw, ClawSide.LEFT);
        updateState(rightClaw, ClawSide.RIGHT);
    }

    public void setAngle(@NotNull Angle angle) {
        this.angle = angle;

        updateState(Type.HAND);
        updateState(Type.CLAW);
    }

    public void updateState(@NotNull Type type) {

        switch(type) {
            case CLAW:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(intakeClawPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(outtakeClawPivot);
                        break;
                    case OUTTAKE_LONG:
                        this.robot.outtakeClawPivotServo.setPosition(outtakeClawPivotLong);
                        break;
                }
                break;
            case HAND:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeHandRightServo.setPosition(intakeHandPivot);
                        this.robot.outtakeHandLeftServo.setPosition(intakeHandPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeHandRightServo.setPosition(outtakeHandPivot);
                        this.robot.outtakeHandLeftServo.setPosition(outtakeHandPivot);
                        break;
                    case OUTTAKE_LONG:
                        this.robot.outtakeHandRightServo.setPosition(outtakeHandPivotLong);
                        this.robot.outtakeHandLeftServo.setPosition(outtakeHandPivotLong);
                        break;
                }
                break;
        }
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