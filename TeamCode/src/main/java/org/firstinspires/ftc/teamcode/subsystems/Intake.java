package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Intake extends BetterSubsystem {

    private final RobotHardware robot;

    //public static double intakeAngle1 = 0.07, intakeAngle2 = 0, intakeAngle3 = 0;
    public static double intakePivotAngle1 = 0.2, intakePivotAngle2 = 0.3, intakePivotAngle3 = 0.4;

    public static double power = 1;
    public static double handToAmmoRatio = 0.3;
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

    public Intake()
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
        else
        {
            robot.intakeServoRight.setPower(0);
            robot.intakeServoLeft.setPower(0);
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

    public void setAmmo(Angle ammo) {
        this.ammo = ammo;
    }

    public void setHand(Angle hand) {
        this.hand = hand;
    }

    public void updateState(@NotNull Angle angle, @NotNull Type type) {
        double position = getPosition(angle, type);

        switch(type) {
            case AMMO:
                this.robot.intakeAngleServo.setPosition(position);
                break;
            case HAND:
                this.robot.intakeHandPivotLeftServo.setPosition(position);
                this.robot.intakeHandPivotRightServo.setPosition(position);
                break;
        }


    }

    private double getPosition(Angle angle, Type type)
    {
        switch (type)
        {
            case AMMO:
                switch (angle) {
                    case INTAKE:
                        return intakePivotAngle1 * handToAmmoRatio;
                    case MID:
                        return intakePivotAngle2 * handToAmmoRatio;
                    case TRANSFER:
                        return intakePivotAngle3 * handToAmmoRatio;
                    default:
                        return 0.0;

                }

            case HAND:
                switch (angle) {
                    case INTAKE:
                        return intakePivotAngle1 ;
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

    public boolean isShouldIntake() {
        return shouldIntake;
    }

    public void setShouldIntake(boolean shouldIntake) {
        this.shouldIntake = shouldIntake;
    }

    public boolean isForward() {
        return forward;
    }

    public void setForward(boolean forward) {
        this.forward = forward;
    }
}