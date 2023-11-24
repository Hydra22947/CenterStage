package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Intake extends BetterSubsystem {

    private final RobotHardware robot;

    public static double intakeHandPivot = 0.08, intakeAmmoPivot = 0.04;
    public static double outtakeHandPivot = 0.5, outtakeAmmoPivot = 0.58;

    public static double power = 1;
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
    boolean manual = false;

    public Intake()
    {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        updateState(ammo, Type.AMMO);
        updateState(hand, Type.HAND);

        if(shouldIntake && forward && !manual)
        {
            robot.intakeServoRight.setPower(power);
            robot.intakeServoLeft.setPower(power);
        }
        else if(shouldIntake && !manual)
        {
            robot.intakeServoRight.setPower(-power);
            robot.intakeServoLeft.setPower(-power);
        }
        else if(!manual)
        {
            robot.intakeServoRight.setPower(0);
            robot.intakeServoLeft.setPower(0);
        }

    }

    public void intakeMove(double power)
    {
        robot.intakeServoRight.setPower(power);
        robot.intakeServoLeft.setPower(power);
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
                        return intakeAmmoPivot;
                    case TRANSFER:
                        return outtakeAmmoPivot;
                    default:
                        return 0.0;

                }

            case HAND:
                switch (angle) {
                    case INTAKE:
                        return intakeHandPivot;
                    case TRANSFER:
                        return outtakeHandPivot;
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

    public void setManual(boolean manual) {
        this.manual = manual;
    }

}