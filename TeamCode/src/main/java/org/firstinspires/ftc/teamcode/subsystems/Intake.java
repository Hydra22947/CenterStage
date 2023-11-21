package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Intake extends BetterSubsystem {

    private final RobotHardware robot;

    public static double intakePivot = 0.065, outtakePivot = 0.58;

    public static double power = 1;
    public static double handToAmmoRatio = 1.15;
    public enum Angle
    {
        INTAKE,
        TRANSFER
    }

    public enum Type
    {
        AMMO,
        HAND
    }

    public Angle ammo = Angle.TRANSFER;
    public Angle hand = Angle.TRANSFER;

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
                        return intakePivot * handToAmmoRatio;
                    case TRANSFER:
                        return outtakePivot * handToAmmoRatio;
                    default:
                        return 0.0;

                }

            case HAND:
                switch (angle) {
                    case INTAKE:
                        return intakePivot ;
                    case TRANSFER:
                        return outtakePivot;
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