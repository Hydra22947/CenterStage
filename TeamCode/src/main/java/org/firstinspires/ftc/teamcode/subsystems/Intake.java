package org.firstinspires.ftc.teamcode.subsystems;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Intake extends BetterSubsystem {

    private final RobotHardware robot;

    public static double intakeHandPivot = 0.06, intakeAmmoPivot = 0.04;
    public static double outtakeHandPivot = 0.5, outtakeAmmoPivot = 0.58;

    public static double power = 1;
    public static double powerTransfer = -0.1;
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

    Angle angle = Angle.TRANSFER;
    boolean shouldIntake = false;
    boolean forward = false;
    boolean manual = false;

    public Intake()
    {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
        updateState(Type.AMMO);
        updateState(Type.HAND);

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

//        if(hasPixel(robot.colorLeft) && hasPixel(robot.colorRight) && getAngle() == Angle.TRANSFER)
//        {
//            robot.setReadyToTransferPixels(true);
//        }

    }

    boolean hasPixel(RevColorSensorV3 colorSensor)
    {
        return false;
    }

    public void intakeMove(double power)
    {
        robot.intakeServoRight.setPower(power);
        robot.intakeServoLeft.setPower(power);
    }

    public void move(Angle angle)
    {
        setAngle(angle);

        updateState(Type.AMMO);
        updateState(Type.HAND);
    }


    public Angle getAngle() {
        return angle;
    }

    public void setAngle(Angle angle) {
        this.angle = angle;
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

    public void updateState(@NotNull Type type) {
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
    public void finishedIntake()
    {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (this.robot.colorRight.red() * SCALE_FACTOR),
                (int) (this.robot.colorRight.green() * SCALE_FACTOR),
                (int) (this.robot.colorRight.blue() * SCALE_FACTOR),
                hsvValues);
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