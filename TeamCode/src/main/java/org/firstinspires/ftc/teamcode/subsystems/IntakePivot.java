package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.values.ClawSide;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class IntakePivot extends BetterSubsystem {

    private final RobotHardware robot;

    public static double intakeAngle1 = 0, intakeAngle2 = 0, intakeAngle3 = 0;
    public static double intakePivotAngle1 = 0, intakePivotAngle2 = 0, intakePivotAngle3 = 0;

    public static double power = 1;
    public enum Angle
    {
        INTAKE,
        MID,
        TRANSFER
    }

    public enum Type
    {
        AMMO,
        HAND
    }

    public Angle ammo = Angle.INTAKE;
    public Angle hand = Angle.INTAKE;

    boolean shouldIntake = false;
    boolean forward = false;

    public IntakePivot()
    {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        updateState(ammo, Type.AMMO);
        updateState(hand, Type.HAND);

        if(shouldIntake && forward)
        {
            robot.intakeServoRight.setPower(power);
            robot.intakeServoLeft.setPower(power);
        }
        else if(shouldIntake)
        {
            robot.intakeServoRight.setPower(-power);
            robot.intakeServoLeft.setPower(-power);
        }

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

    public void updateState(@NotNull Angle angle, @NotNull Type type) {
        double position = getClawStatePosition(angle, type);

        switch(type) {
            case AMMO:
                this.robot.intakeAngleServo.setPosition(position);
            case HAND:
                this.robot.intakeHandPivotLeftServo.setPosition(position);
                this.robot.intakeHandPivotRightServo.setPosition(position);
                break;
        }


    }

    private double getClawStatePosition(Angle angle, Type type)
    {
        switch (type)
        {
            case AMMO:
                switch (angle) {
                    case INTAKE:
                        return intakeAngle1;
                    case MID:
                        return intakeAngle2;
                    case TRANSFER:
                        return intakeAngle3;
                    default:
                        return 0.0;
                }
            case HAND:
                switch (angle) {
                    case INTAKE:
                        return intakePivotAngle1;
                    case MID:
                        return intakePivotAngle2;
                    case TRANSFER:
                        return intakePivotAngle3;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }
}