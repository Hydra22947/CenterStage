package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.jetbrains.annotations.NotNull;

@Config
public class Intake {

    private final RobotHardware robot;
    public static double intakeHandPivot = 0.07, intakeAmmoPivot = 0.19;
    public static double outtakeHandPivot = .55, outtakeAmmoPivot = 0.76;
    public static double midHandPivot = 0.6, midAmmoPivot = 0.2;

    public static double openRight = 0.55, closeRight = 0.37;
    public static double openLeft = 0.45, closeLeft = 0.63;

    public enum Angle
    {
        INTAKE,
        TRANSFER,
        MID
    }

    public enum ClawState
    {
        OPEN,
        CLOSE
    }

    public enum Type
    {
        AMMO,
        HAND,
        CLAW
    }

    Angle angle = Angle.TRANSFER;
    ClawState clawStateLeft = ClawState.OPEN;
    ClawState clawStateRight = ClawState.OPEN;


    public Intake()
    {
        this.robot = RobotHardware.getInstance();
    }

    public void update() {
        updateState(Type.AMMO);
        updateState(Type.HAND);


        if(checkIfPixelIn(robot.colorRight))
        {
            robot.closeRight(true);
        }
        else
        {
            robot.closeRight(false);
        }

        if(checkIfPixelIn(robot.colorLeft))
        {
            robot.closeLeft(true);
        }
        else
        {
            robot.closeLeft(false);
        }


        if(checkIfPixelIn(robot.colorRight) && checkIfPixelIn(robot.colorLeft))
        {
            robot.setHas2Pixels(true);
        }
        else
        {
            robot.setHas2Pixels(false);
        }
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

    public void updateClawState(@NotNull ClawState state, @NotNull ClawSide side) {
        double position = getClawStatePosition(state, side);

        switch (side) {
            case LEFT:
                robot.intakeClawLeftServo.setPosition(position);
                this.clawStateLeft = state;
                break;
            case RIGHT:
                robot.intakeClawRightServo.setPosition(position);
                this.clawStateRight = state;
                break;
            case BOTH:
                position = getClawStatePosition(state, ClawSide.LEFT);
                robot.intakeClawLeftServo.setPosition(position);
                this.clawStateRight = state;
                position = getClawStatePosition(state, ClawSide.RIGHT);
                robot.intakeClawRightServo.setPosition(position);
                this.clawStateLeft = state;
                break;
        }
    }

    private double getClawStatePosition(ClawState state, ClawSide side)
    {
        switch (side)
        {
            case LEFT:
                switch (state) {
                    case CLOSE:
                        return closeLeft;
                    case OPEN:
                        return openLeft;
                    default:
                        return 0.0;
                }
            case RIGHT:
                switch (state) {
                    case CLOSE:
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


    public boolean checkIfPixelIn(RevColorSensorV3 sensor)
    {
        return -Globals.ERROR <= sensor.getDistance(DistanceUnit.CM) && sensor.getDistance(DistanceUnit.CM) <= Globals.ERROR;
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
                    case MID:
                        return midAmmoPivot;
                    default:
                        return 0.0;

                }

            case HAND:
                switch (angle) {
                    case INTAKE:
                        return intakeHandPivot;
                    case TRANSFER:
                        return outtakeHandPivot;
                    case MID:
                        return midHandPivot;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }


}