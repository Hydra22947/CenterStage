package org.firstinspires.ftc.teamcode.subsystems;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ColorSensors;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Intake {

    private final RobotHardware robot;
    private ColorSensors sensors;
    public static double intakeHandPivot = 0.05, intakeAmmoPivot = 0;
    public static double outtakeHandPivot = 0.47, outtakeAmmoPivot = 0.56;

    public static double power = 1;
    public static double powerTransfer = -0.1;
    public static double ERROR = 3;
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

    Angle angle = Angle.TRANSFER;
    boolean shouldIntake = false;
    boolean forward = false;
    boolean manual = false;

    private float rightHsvValues[] = {0F, 0F, 0F};
    private float leftHsvValues[] = {0F, 0F, 0F};

    public Intake()
    {
        this.robot = RobotHardware.getInstance();
        this.sensors = new ColorSensors();
    }

    public void update() {
        insertHSVValues();

        updateState(Type.AMMO);
        updateState(Type.HAND);
        //checkFinishedIntake();

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

        if(checkIfPixelIn(leftHsvValues) && checkIfPixelIn(rightHsvValues))
        {
            robot.setReadyToRetract(true);
        }
        else
        {
            robot.setReadyToRetract(false);
        }
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

    public void insertHSVValues()
    {
        Color.RGBToHSV((int) (this.robot.colorRight.red() * Globals.SCALE_FACTOR),
                (int) ((this.robot.colorRight.green() * Globals.SCALE_FACTOR)),
                (int) ((this.robot.colorRight.blue() * Globals.SCALE_FACTOR)),
                rightHsvValues);

        Color.RGBToHSV((int) (this.robot.colorLeft.red() * Globals.SCALE_FACTOR),
                (int) (this.robot.colorLeft.green() * Globals.SCALE_FACTOR),
                (int) (this.robot.colorLeft.blue() * Globals.SCALE_FACTOR),
                leftHsvValues);
    }

    public boolean checkIfPixelIn(float[] hsvValues)
    {
        boolean hueCheck = ((hsvValues[0] + ERROR) == Globals.BASIC_HUE) || ((hsvValues[0] - ERROR) == Globals.BASIC_HUE);
        boolean satCheck = ((hsvValues[1] + ERROR) == Globals.BASIC_HUE) || ((hsvValues[1] - ERROR) == Globals.BASIC_HUE);
        boolean valCheck = ((hsvValues[2] + ERROR) == Globals.BASIC_HUE) || ((hsvValues[2] - ERROR) == Globals.BASIC_HUE);

        return hueCheck &&  satCheck && valCheck;
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
    public boolean checkFinishedIntake()
    {
        return this.sensors.checkIfPixelIn(this.sensors.getRightHsvValues()) || this.sensors.checkIfPixelIn(this.sensors.getLeftHsvValues());

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

    public static double getPower() {
        return power;
    }
}