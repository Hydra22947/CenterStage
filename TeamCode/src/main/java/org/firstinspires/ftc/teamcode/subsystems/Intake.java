package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.jetbrains.annotations.NotNull;

@Config
public class Intake implements Subsystem{

    private final RobotHardware robot;
    public static double intakeHandPivot = 0.185, intakeAmmoPivot = 0.09;
    public static double outtakeHandPivot = .65, outtakeAmmoPivot = .64;
    public static double midHandPivot = 0.61, midAmmoPivot = 0.375;
    public static double top5HandPivot = .28, top5AmmoPivot = 0.01; // auto
    public static double top54HandPivot = 0.261, top54AmmoPivot = 0.03;
    public static double top54HandPivotAuto = 0.235, top54AmmoPivotAuto = 0.035;
    public static double top43HandPivot = 0.225, top43AmmoPivot = 0.0185; // auto
    public static double top32HandPivot = 0.235, top32AmmoPivot = 0.03;
    public static double top32HandPivotAuto = 0.18, top32AmmoPivotAuto = 0.025;
    public static double top21HandPivot = 0.185, top21AmmoPivot = 0.01; // auto

    public static double STACK = 0.02;
    public static double RIGHT_SENSOR_ERROR = 0.3;

    public static double openRight = 0.55, openLeft = 0.525;
    public static double closeRight = .74, closeLeft = .73;
    public static double indeterminateRight = 0.67, indeterminateLeft = 0.65;


    public static void setSeeFarFrom(double seeFarFrom) {
        Intake.seeFarFrom = seeFarFrom;
    }

    public static double seeFarFrom = 2.7;
    public static final double maxSeeFarFrom = 2.7;
    public static final double minSeeFarFrom = 1;

    @Override
    public void play() {

    }

    @Override
    public void loop(boolean allowMotors) {
        updateState(Type.AMMO);
        updateState(Type.HAND);
    }

    @Override
    public void stop() {

    }

    public enum Angle
    {
        INTAKE,
        OUTTAKE,
        MID,
        TOP_54, TOP_43, TOP_32, TOP_21, TOP_5,
        TOP_54_AUTO, TOP_32_AUTO
    }

    public enum ClawState
    {
        OPEN,
        INDETERMINATE,
        CLOSE
    }

    public enum Type
    {
        AMMO,
        HAND
    }

    Angle angle = Angle.OUTTAKE;
    ClawState clawStateLeft = ClawState.OPEN;
    ClawState clawStateRight = ClawState.OPEN;


    public Intake()
    {
        this.robot = RobotHardware.getInstance();
    }

    public void update() {
        updateState(Type.AMMO);
        updateState(Type.HAND);


        if(checkIfPixelInRight(robot.colorRight))
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


        if(checkIfPixelInRight(robot.colorRight) && checkIfPixelIn(robot.colorLeft))
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

    public void moveStack()
    {
        this.robot.intakeAngleServo.setPosition((getPosition(angle, Type.AMMO) + STACK));
        this.robot.intakeHandPivotLeftServo.setPosition(getPosition(angle, Type.HAND));
        this.robot.intakeHandPivotRightServo.setPosition(getPosition(angle, Type.HAND));
    }



    public Angle getAngle() {
        return angle;
    }

    public void setAngle(Angle angle) {
        this.angle = angle;

        updateState(Type.AMMO);
        updateState(Type.HAND);
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

    public double updateClawState(@NotNull ClawState state, @NotNull ClawSide side) {
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

        return position;
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
                    case INDETERMINATE:
                        return indeterminateLeft;
                    default:
                        return 0.0;
                }
            case RIGHT:
                switch (state) {
                    case CLOSE:
                        return closeRight;
                    case OPEN:
                        return openRight;
                    case INDETERMINATE:
                        return indeterminateRight;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }

    public ClawState getClawStateLeft() {
        return clawStateLeft;
    }

    public ClawState getClawStateRight() {
        return clawStateRight;
    }

    public boolean checkIfPixelIn(RevColorSensorV3 sensor)
    {
        return -seeFarFrom <= sensor.getDistance(DistanceUnit.CM) && sensor.getDistance(DistanceUnit.CM) <= seeFarFrom;
    }

    public boolean checkIfPixelInRight(RevColorSensorV3 sensor)
    {
        return -(seeFarFrom+RIGHT_SENSOR_ERROR) <= sensor.getDistance(DistanceUnit.CM) && sensor.getDistance(DistanceUnit.CM) <= (seeFarFrom+RIGHT_SENSOR_ERROR);
    }

    public boolean closedClaw()
    {
        return clawStateLeft == ClawState.CLOSE && clawStateRight == ClawState.CLOSE;
    }
    private double getPosition(Angle angle, Type type)
    {
        switch (type)
        {
            case AMMO:
                switch (angle) {
                    case INTAKE:
                        return intakeAmmoPivot;
                    case OUTTAKE:
                        return outtakeAmmoPivot;
                    case MID:
                        return midAmmoPivot;
                    case TOP_21:
                        return top21AmmoPivot;
                    case TOP_54:
                        return top54AmmoPivot;
                    case TOP_32:
                        return top32AmmoPivot;
                    case TOP_5:
                        return top5AmmoPivot;
                    case TOP_43:
                        return top43AmmoPivot;
                    case TOP_54_AUTO:
                        return top54AmmoPivotAuto;
                    case TOP_32_AUTO:
                        return top32AmmoPivotAuto;
                    default:
                        return 0.0;

                }

            case HAND:
                switch (angle) {
                    case INTAKE:
                        return intakeHandPivot;
                    case OUTTAKE:
                        return outtakeHandPivot;
                    case MID:
                        return midHandPivot;
                    case TOP_21:
                        return top21HandPivot;
                    case TOP_32:
                        return top32HandPivot;
                    case TOP_54:
                        return top54HandPivot;
                    case TOP_5:
                        return top5HandPivot;
                    case TOP_43:
                        return top43HandPivot;
                    case TOP_54_AUTO:
                        return top54HandPivotAuto;
                    case TOP_32_AUTO:
                        return top32HandPivotAuto;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }


}